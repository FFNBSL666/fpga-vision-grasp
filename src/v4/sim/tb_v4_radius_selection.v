`timescale 1ns/1ps
module tb_v4_radius_selection;
    reg clk=0,rst_n=0,enable=0,result_valid=0;
    reg [49:0] detections=0;
    always #5 clk=~clk;
    wire response,valid;
    wire [1:0] phase;
    wire [15:0] source_pixel,destination_pixel;
    wire [95:0] sa,sd,sg,sl,da,dd,dg,dl;
    pick_place_target_mapper dut(.clk(clk),.rst_n(rst_n),.enable(enable),
        .result_valid(result_valid),.detections(detections),
        .response(response),.valid(valid),.phase(phase),
        .source_pixel(source_pixel),.destination_pixel(destination_pixel),
        .source_approach(sa),.source_descent(sd),.source_grip(sg),.source_lift(sl),
        .destination_approach(da),.destination_descent(dd),
        .destination_release(dg),.destination_retreat(dl));
    integer accepted=0,checks=0,dx,dy,quadrant,distance_squared;
    reg expected;
    reg [7:0] fx,fy,ox,oy;
    reg [95:0] saved_sa,saved_sd;
    always @(posedge clk)if(rst_n && response && valid)accepted=accepted+1;
    task clocks;
        input integer n;integer i;
        begin for(i=0;i<n;i=i+1)@(negedge clk);end
    endtask
    task sample;
        input finger,object_present;
        input [7:0] finger_x,finger_y,object_x,object_y;
        begin
            detections={object_y,object_x,8'd240,object_present,
                        finger_y,finger_x,8'd240,finger};
            result_valid=1;clocks(1);result_valid=0;clocks(10);
        end
    endtask
    initial begin
        clocks(4);rst_n=1;
        // Cover every possible valid delta in all four directions, exercising
        // multiplication widths and the anisotropic physical plane scale.
        for(quadrant=0;quadrant<4;quadrant=quadrant+1)
            for(dy=0;dy<96;dy=dy+1)for(dx=0;dx<96;dx=dx+1)begin
                ox=(quadrant&1)?95:0;fx=(quadrant&1)?95-dx:dx;
                oy=(quadrant&2)?95:0;fy=(quadrant&2)?95-dy:dy;
                detections={oy,ox,8'd240,1'b1,fy,fx,8'd240,1'b1};clocks(1);
                distance_squared=dx*dx*48400+dy*dy*22500;
                expected=distance_squared<=8122500;
                if(dut.source_eligible!==expected)$fatal(1,"CIRCLE_MISMATCH_%0d_%0d_%0d",quadrant,dx,dy);
                checks=checks+1;
            end
        enable=1;clocks(3);
        repeat(5)sample(0,1,48,48,48,48);
        repeat(5)sample(1,0,48,48,48,48);
        repeat(5)sample(1,1,255,48,48,48);
        repeat(5)sample(1,1,48,48,255,48);
        repeat(5)sample(1,1,61,48,48,48); // 30.105 mm, outside
        repeat(5)sample(1,1,48,68,48,48); // 31.579 mm, outside
        repeat(5)sample(1,1,58,61,48,48); // diagonal, outside true circle
        if(phase!=0 || accepted)$fatal(1,"INVALID_OR_OUTSIDE_LOCKED");
        sample(1,1,48,67,48,48);sample(1,1,48,67,48,48); // exactly 30 mm
        if(phase!=0)$fatal(1,"LOCKED_BEFORE_THREE_FRAMES");
        sample(1,1,48,68,48,48); // interrupt stability
        sample(1,1,48,67,48,48);sample(1,1,48,67,48,48);
        if(phase!=0)$fatal(1,"INTERRUPTION_NOT_RESET");
        sample(1,1,48,67,48,48);
        if(phase!=1 || source_pixel!={8'd48,8'd48} || accepted)$fatal(1,"BOUNDARY_OR_OBJECT_CENTER_LOCK");
        saved_sa=sa;saved_sd=sd;
        repeat(5)sample(1,1,5,5,48,48);
        if(phase!=1 || accepted)$fatal(1,"ONE_APPEARANCE_COUNTED_TWICE");
        sample(0,1,0,0,48,48);
        repeat(3)sample(1,1,5,5,48,48);
        if(phase!=1 || accepted)$fatal(1,"ONE_CLEAR_FRAME_ACCEPTED");
        repeat(2)sample(0,1,0,0,48,48);
        repeat(4)sample(1,0,5,5,48,48);
        if(phase!=1 || accepted)$fatal(1,"DESTINATION_OBJECT_REQUIREMENT_CHANGED");
        repeat(3)sample(1,1,5,5,48,48); // second point may be far from object
        if(phase!=2 || destination_pixel!={8'd5,8'd5} || accepted)$fatal(1,"SECOND_SELECTION_CHANGED");
        if(source_pixel!={8'd48,8'd48} || sa!==saved_sa || sd!==saved_sd)$fatal(1,"SOURCE_POSE_CHANGED");
        repeat(4)sample(1,1,5,5,48,48);
        if(accepted)$fatal(1,"EXECUTION_BEFORE_WITHDRAWAL");
        sample(0,1,0,0,48,48);
        if(accepted)$fatal(1,"EXECUTION_BEFORE_TWO_CLEAR_FRAMES");
        sample(0,1,0,0,48,48);
        if(accepted!=1)$fatal(1,"NO_EXECUTION_AFTER_SECOND_WITHDRAWAL");
        repeat(5)sample(1,1,48,48,48,48);
        if(accepted!=1)$fatal(1,"REPEATED_EXECUTION");
        enable=0;clocks(3);enable=1;clocks(3);
        if(phase!=0 || source_pixel!=0 || destination_pixel!=0)$fatal(1,"RESET_CHANGED");
        repeat(3)sample(1,1,57,61,48,48); // diagonal inside
        if(phase!=1)$fatal(1,"INSIDE_DIAGONAL_REJECTED");
        $display("PASS: V4_RADIUS_30MM_%0d_OFFSETS_BOUNDARY_STABILITY_TWO_POINTS_UNCHANGED",checks);
        $finish;
    end
    initial begin #2000000;$fatal(1,"TIMEOUT");end
endmodule
