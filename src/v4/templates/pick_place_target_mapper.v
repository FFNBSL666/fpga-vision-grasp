`timescale 1ns/1ps
// Presence-triggered pick/place selection; this does not infer pointing direction.
// Lock object on first stable joint presence, then require a hand withdrawal.
// Lock finger on second joint presence; execute only after the second withdrawal.
module pick_place_target_mapper(
    input wire clk,rst_n,enable,result_valid,
    input wire [49:0] detections,
    output reg response,valid,
    output reg [1:0] phase,
    output reg [15:0] source_pixel,destination_pixel,
    output reg [95:0] source_approach,source_descent,source_grip,source_lift,
    output reg [95:0] destination_approach,destination_descent,destination_release,destination_retreat
);
    localparam [2:0] SOURCE=0,SOURCE_MAP=1,FIRST_CLEAR=2,
                     DESTINATION=3,DESTINATION_MAP=4,SECOND_CLEAR=5,DONE=6;
    reg [2:0] state;
    reg [1:0] point_count,clear_count;
    reg [7:0] candidate_x,candidate_y;
    reg sr,dr;
    wire sresponse,svalid,dresponse,dvalid;
    wire [7:0] fx=detections[16:9],fy=detections[24:17];
    wire [7:0] ox=detections[41:34],oy=detections[49:42];
    wire finger=detections[0] && detections[8:1]>=128 && fx<96 && fy<96;
    wire object_ok=detections[25] && detections[33:26]>=128 && ox<96 && oy<96;
    wire both_present=finger && object_ok;
    wire [7:0] source_dx=ox>source_pixel[7:0]?ox-source_pixel[7:0]:source_pixel[7:0]-ox;
    wire [7:0] source_dy=oy>source_pixel[15:8]?oy-source_pixel[15:8]:source_pixel[15:8]-oy;
    wire same_object=object_ok && source_dx<=6 && source_dy<=6;
    wire [7:0] selected_x=state==SOURCE?ox:fx;
    wire [7:0] selected_y=state==SOURCE?oy:fy;
    wire [7:0] stable_dx=selected_x>candidate_x?selected_x-candidate_x:candidate_x-selected_x;
    wire [7:0] stable_dy=selected_y>candidate_y?selected_y-candidate_y:candidate_y-selected_y;
    wire [95:0] sa,sd,sg,sl,da,dd,dg,dl;
    pixel_pose_lookup source_lookup(
        .clk(clk),.rst_n(rst_n),.request(sr),.pixel_x(source_pixel[7:0]),.pixel_y(source_pixel[15:8]),
        .response(sresponse),.valid(svalid),.approach_pose(sa),.descent_pose(sd),.grip_pose(sg),.lift_pose(sl));
    pixel_pose_lookup #(.MEM_FILE("placement_pose_lut.mem")) destination_lookup(
        .clk(clk),.rst_n(rst_n),.request(dr),.pixel_x(destination_pixel[7:0]),.pixel_y(destination_pixel[15:8]),
        .response(dresponse),.valid(dvalid),.approach_pose(da),.descent_pose(dd),.grip_pose(dg),.lift_pose(dl));
    always @(posedge clk) begin
        if(!rst_n || !enable)begin
            state<=SOURCE;phase<=0;response<=0;valid<=0;sr<=0;dr<=0;
            point_count<=0;clear_count<=0;candidate_x<=0;candidate_y<=0;
            source_pixel<=0;destination_pixel<=0;
            source_approach<=0;source_descent<=0;source_grip<=0;source_lift<=0;
            destination_approach<=0;destination_descent<=0;destination_release<=0;destination_retreat<=0;
        end else begin
            response<=0;valid<=0;sr<=0;dr<=0;
            if(state==SOURCE_MAP && sresponse)begin
                if(svalid)begin
                    source_approach<=sa;source_descent<=sd;source_grip<=sg;source_lift<=sl;
                    phase<=1;state<=FIRST_CLEAR;clear_count<=0;
                end else begin state<=SOURCE;point_count<=0;response<=1;end
            end
            if(state==DESTINATION_MAP && dresponse)begin
                if(dvalid)begin
                    destination_approach<=dl;destination_descent<=dg;
                    destination_release<=dd;destination_retreat<=da;
                    phase<=2;state<=SECOND_CLEAR;clear_count<=0;
                end else begin state<=DESTINATION;point_count<=0;response<=1;end
            end
            if(result_valid)case(state)
                SOURCE,DESTINATION:begin
                    if(!both_present || (state==DESTINATION && !same_object))point_count<=0;
                    else if(point_count==0 || stable_dx>6 || stable_dy>6)begin
                        candidate_x<=selected_x;candidate_y<=selected_y;point_count<=1;
                    end else if(point_count==2)begin
                        point_count<=0;
                        if(state==SOURCE)begin
                            source_pixel<={oy,ox};sr<=1;state<=SOURCE_MAP;
                        end else begin
                            destination_pixel<={fy,fx};dr<=1;state<=DESTINATION_MAP;
                        end
                    end else point_count<=point_count+1'b1;
                end
                FIRST_CLEAR:begin
                    if(finger)clear_count<=0;
                    else if(clear_count==1)begin
                        state<=DESTINATION;clear_count<=0;point_count<=0;
                    end else clear_count<=clear_count+1'b1;
                end
                SECOND_CLEAR:begin
                    if(finger || !same_object)clear_count<=0;
                    else if(clear_count==1)begin
                        response<=1;valid<=1;state<=DONE;clear_count<=0;
                    end else clear_count<=clear_count+1'b1;
                end
                default:begin end
            endcase
        end
    end
endmodule
