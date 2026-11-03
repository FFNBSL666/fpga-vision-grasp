`timescale 1ns/1ps
module tb_presence_selection;
    reg clk=0,rst_n=0,enable=0,result_valid=0;
    reg [49:0] detections=0;
    always #10 clk=~clk;
    wire response,valid;
    wire [1:0] phase;
    wire [15:0] source_pixel,destination_pixel;
    wire [95:0] sa,sd,sg,sl,da,dd,dg,dl;
    pick_place_target_mapper dut(.clk(clk),.rst_n(rst_n),.enable(enable),.result_valid(result_valid),
        .detections(detections),.response(response),.valid(valid),.phase(phase),
        .source_pixel(source_pixel),.destination_pixel(destination_pixel),
        .source_approach(sa),.source_descent(sd),.source_grip(sg),.source_lift(sl),
        .destination_approach(da),.destination_descent(dd),.destination_release(dg),.destination_retreat(dl));
    reg [49:0] recorded[0:9];
    integer i,accepted=0;
    always @(posedge clk)if(rst_n && response && valid)accepted=accepted+1;
    task clocks;
        input integer n;integer j;
        begin for(j=0;j<n;j=j+1)@(negedge clk);end
    endtask
    task sample;
        input [49:0] value;
        begin detections=value;result_valid=1;clocks(1);result_valid=0;clocks(10);end
    endtask
    task synthetic;
        input fv,ov;
        input [7:0] fx,fy;
        begin sample({8'd43,8'd18,8'd240,ov,fy,fx,8'd240,fv});end
    endtask
    initial begin
        $readmemh("recorded_presence.mem",recorded);
        clocks(5);rst_n=1;enable=1;clocks(5);
        // Actual prior model outputs had finger-object separations >20 pixels.
        for(i=0;i<3;i=i+1)sample(recorded[i]);
        if(phase!=1 || source_pixel!={8'd43,8'd18} || accepted)$fatal(1,"RECORDED_JOINT_PRESENCE_NOT_ACCEPTED");
        for(i=3;i<10;i=i+1)sample(recorded[i]);
        if(phase!=1 || accepted)$fatal(1,"ONE_APPEARANCE_COUNTED_TWICE");
        synthetic(0,1,0,0);synthetic(0,1,0,0);
        repeat(5)synthetic(1,0,80,60);
        if(phase!=1 || accepted)$fatal(1,"DESTINATION_MISSING_OBJECT");
        repeat(3)synthetic(1,1,80,60);
        if(phase!=2 || destination_pixel!={8'd60,8'd80} || accepted)$fatal(1,"SECOND_LOCK_OR_LED_PHASE");
        repeat(5)synthetic(1,1,80,60);
        if(phase!=2 || accepted)$fatal(1,"EXECUTED_BEFORE_SECOND_WITHDRAWAL");
        synthetic(0,1,0,0);
        if(accepted)$fatal(1,"ONE_CLEAR_FRAME_WAS_ENOUGH");
        synthetic(0,1,0,0);
        if(accepted!=1)$fatal(1,"HAND_CLEAR_DID_NOT_EXECUTE");
        clocks(100);
        if(accepted!=1)$fatal(1,"AUTOMATIC_REPEAT");
        enable=0;clocks(5);enable=1;clocks(5);
        repeat(5)synthetic(0,1,0,0);
        if(phase!=0 || accepted!=1)$fatal(1,"OBJECT_ALONE_RETRIGGERED");
        $display("PASS: RECORDED_MODEL_PRESENCE_FIRST_LOCK_LED1_SECOND_LOCK_LED2_TWO_WITHDRAWALS_NO_FALSE_REPEAT");
        $finish;
    end
    initial begin #400000;$fatal(1,"TIMEOUT");end
endmodule
