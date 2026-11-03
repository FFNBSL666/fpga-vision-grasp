"""训练一个小型量化网格检测器，并导出 QONNX。

模型输出为 [类别, 5, 网格高, 网格宽]：
  5 个数分别为 objectness、中心点 x、中心点 y、宽、高。
这里只负责训练、量化和 QONNX 导出，不生成 FINN FPGA 加速 IP。
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image
from torch import nn
from torch.utils.data import DataLoader, Dataset

import brevitas.nn as qnn
from brevitas.export import export_qonnx


SEED = 20261017


class DetectionDataset(Dataset):
    def __init__(self, manifest: Path, split: str, image_size: int, grid_size: int, augment: bool) -> None:
        self.rows = []
        for line in manifest.read_text(encoding="utf-8").splitlines():
            if line.strip():
                row = json.loads(line)
                if row["split"] == split:
                    self.rows.append(row)
        self.image_size = image_size
        self.grid_size = grid_size
        self.augment = augment

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        row = self.rows[index]
        image = Image.open(row["image"]).convert("RGB")
        original_w, original_h = image.size
        boxes = [dict(box) for box in row["boxes"]]

        if self.augment and random.random() < 0.5:
            image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
            for box in boxes:
                box["cx"] = 1.0 - box["cx"]

        image = image.resize((self.image_size, self.image_size), Image.Resampling.BILINEAR)
        array = np.asarray(image, dtype=np.float32) / 255.0
        tensor = torch.from_numpy(array).permute(2, 0, 1)
        # 按类别保存一份框；同一类别出现多个框时保留面积最大的一个。
        target = torch.zeros((2, 5, self.grid_size, self.grid_size), dtype=torch.float32)
        target[1, 0, :, :] = -1.0  # 没有目标的 objectness 作为负样本
        for box in boxes:
            class_id = int(box["class_id"])
            if class_id >= 2:
                continue
            gx = min(self.grid_size - 1, max(0, int(float(box["cx"]) * self.grid_size)))
            gy = min(self.grid_size - 1, max(0, int(float(box["cy"]) * self.grid_size)))
            candidate_area = float(box["w"]) * float(box["h"])
            previous = target[class_id, 0, gy, gx]
            if previous > 0 and previous >= candidate_area:
                continue
            target[class_id, 0, gy, gx] = candidate_area
            target[class_id, 1, gy, gx] = float(box["cx"])
            target[class_id, 2, gy, gx] = float(box["cy"])
            target[class_id, 3, gy, gx] = float(box["w"])
            target[class_id, 4, gy, gx] = float(box["h"])
        # 将 objectness 的正样本归一化为 1。
        target[:, 0] = torch.where(target[:, 0] > 0, torch.ones_like(target[:, 0]), target[:, 0])
        return tensor, target, row["relative_image"]


class TinyQNNDetector(nn.Module):
    """适合后续转换的轻量卷积模型，避免使用复杂检测算子。"""

    def __init__(self) -> None:
        super().__init__()
        self.input_quant = qnn.QuantIdentity(bit_width=8, return_quant_tensor=True)
        self.features = nn.Sequential(
            qnn.QuantConv2d(3, 16, kernel_size=3, stride=2, padding=1, bias=False, weight_bit_width=8),
            qnn.QuantReLU(bit_width=8, return_quant_tensor=True),
            nn.MaxPool2d(2),
            qnn.QuantConv2d(16, 32, kernel_size=3, stride=2, padding=1, bias=False, weight_bit_width=8),
            qnn.QuantReLU(bit_width=8, return_quant_tensor=True),
            nn.MaxPool2d(2),
            qnn.QuantConv2d(32, 64, kernel_size=3, stride=1, padding=1, bias=False, weight_bit_width=8),
            qnn.QuantReLU(bit_width=8, return_quant_tensor=True),
            qnn.QuantConv2d(64, 10, kernel_size=1, stride=1, padding=0, bias=True, weight_bit_width=8),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.input_quant(x)
        y = self.features(x)
        return y.reshape(y.shape[0], 2, 5, y.shape[2], y.shape[3])


def detection_loss(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    pred_obj = pred[:, :, 0]
    target_obj = target[:, :, 0]
    positive = target_obj > 0
    negative = ~positive
    obj_loss_pos = nn.functional.binary_cross_entropy_with_logits(
        pred_obj[positive], torch.ones_like(pred_obj[positive]), reduction="sum"
    ) if positive.any() else pred_obj.sum() * 0.0
    obj_loss_neg = nn.functional.binary_cross_entropy_with_logits(
        pred_obj[negative], torch.zeros_like(pred_obj[negative]), reduction="mean"
    ) if negative.any() else pred_obj.sum() * 0.0
    if positive.any():
        # 把回归维度移到最后，才能用 [批次, 类别, 网格行, 网格列] 的正样本掩码索引。
        pred_reg = pred[:, :, 1:5].permute(0, 1, 3, 4, 2)
        target_reg = target[:, :, 1:5].permute(0, 1, 3, 4, 2)
        reg_pred = torch.sigmoid(pred_reg[positive])
        reg_target = target_reg[positive]
        reg_loss = nn.functional.smooth_l1_loss(reg_pred, reg_target, reduction="mean")
    else:
        reg_loss = pred.sum() * 0.0
    return obj_loss_pos / max(1, positive.sum()) + 0.25 * obj_loss_neg + 5.0 * reg_loss


def evaluate(model: nn.Module, loader: DataLoader, device: torch.device) -> dict[str, float]:
    model.eval()
    losses = []
    positives = 0
    true_positive = 0
    with torch.no_grad():
        for images, targets, _ in loader:
            images = images.to(device)
            targets = targets.to(device)
            pred = model(images)
            losses.append(float(detection_loss(pred, targets).cpu()))
            truth = targets[:, :, 0] > 0
            pred_obj = torch.sigmoid(pred[:, :, 0]) > 0.5
            true_positive += int((truth & pred_obj).sum().cpu())
            positives += int(truth.sum().cpu())
    return {
        "loss": float(np.mean(losses)) if losses else math.nan,
        "positive_cells": float(positives),
        "positive_cell_recall": float(true_positive / positives) if positives else 0.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--image-size", type=int, default=96)
    parser.add_argument("--grid-size", type=int, default=6)
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--init-checkpoint", type=Path, default=None)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.set_num_threads(max(1, min(8, torch.get_num_threads())))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    train_set = DetectionDataset(args.manifest, "training", args.image_size, args.grid_size, augment=True)
    test_set = DetectionDataset(args.manifest, "testing", args.image_size, args.grid_size, augment=False)
    if len(train_set) == 0 or len(test_set) == 0:
        raise RuntimeError("训练集或测试集为空")
    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True, num_workers=0)
    test_loader = DataLoader(test_set, batch_size=args.batch_size, shuffle=False, num_workers=0)

    model = TinyQNNDetector().to(device)
    if args.init_checkpoint is not None:
        checkpoint = torch.load(args.init_checkpoint, map_location=device, weights_only=False)
        model.load_state_dict(checkpoint["model"], strict=True)
        print(f"warm_start={args.init_checkpoint}")
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    history: list[dict[str, Any]] = []
    best_loss = float("inf")
    best_path = args.out / "tiny_qnn_detector_best.pt"

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_losses = []
        for images, targets, _ in train_loader:
            images = images.to(device)
            targets = targets.to(device)
            optimizer.zero_grad(set_to_none=True)
            loss = detection_loss(model(images), targets)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            train_losses.append(float(loss.detach().cpu()))
        metrics = evaluate(model, test_loader, device)
        row = {"epoch": epoch, "train_loss": float(np.mean(train_losses)), **metrics}
        history.append(row)
        print(json.dumps(row, ensure_ascii=False))
        if metrics["loss"] < best_loss:
            best_loss = metrics["loss"]
            torch.save({"model": model.state_dict(), "args": vars(args), "classes": ["finger_tip", "object"]}, best_path)

    # 即使后续导出环境缺少依赖，也先把训练记录保存下来。
    (args.out / "training_history.json").write_text(json.dumps(history, ensure_ascii=False, indent=2), encoding="utf-8")
    model.load_state_dict(torch.load(best_path, map_location=device, weights_only=False)["model"])
    model.eval().cpu()
    dummy = torch.zeros(1, 3, args.image_size, args.image_size)
    qonnx_path = args.out / "tiny_qnn_detector_qonnx.onnx"
    export_qonnx(
        model,
        input_t=dummy,
        export_path=str(qonnx_path),
        dynamo=False,
        opset_version=13,
        keep_initializers_as_inputs=False,
    )
    torch.save(model.state_dict(), args.out / "tiny_qnn_detector_best_state.pt")
    summary = {
        "device": str(device),
        "train_samples": len(train_set),
        "test_samples": len(test_set),
        "image_size": args.image_size,
        "grid_size": args.grid_size,
        "classes": ["finger_tip", "object"],
        "best_test_loss": best_loss,
        "qonnx": str(qonnx_path),
    }
    (args.out / "training_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
