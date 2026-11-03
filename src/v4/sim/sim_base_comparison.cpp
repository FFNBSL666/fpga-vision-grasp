#include "Vconfirmed_grasp_system.h"
#include "verilated.h"
#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
static std::vector<uint64_t> readhex(const std::string&p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p);std::vector<uint64_t>v;std::string s;while(f>>s)v.push_back(std::stoull(s,nullptr,16));return v;}
using Pose=std::array<unsigned,6>;
static std::vector<unsigned> expected_base;
static Pose last_pose{};
static std::array<bool,6> known{};
static size_t initial_packet_bytes=0,expected_joint_commands=0;
static unsigned packet_stage=0;
static std::string fixture_folder;
static std::string packet(const Pose&p){
    auto bytes=readhex(fixture_folder+"/expected_stage"+std::to_string(packet_stage)+".mem");
    std::string result;for(auto b:bytes)result.push_back(char(b));
    for(size_t i=0;i<result.size();i+=15){
        unsigned id=999,pulse=0,t=0;
        if(sscanf(result.c_str()+i,"#%3uP%4uT%4u!",&id,&pulse,&t)!=3)throw std::runtime_error("Bad fixture");
        if(id==0)expected_base.push_back(pulse);expected_joint_commands++;
    }
    if(packet_stage==0)initial_packet_bytes=result.size();packet_stage++;return result;
}
int main(int argc,char**argv){
    Verilated::commandArgs(argc,argv);if(argc<2)return 2;
    try{
        bool stop=argc>2 && std::string(argv[2])=="estop";
        bool capture=argc>2 && std::string(argv[2])=="capture";std::string folder=argv[1];fixture_folder=folder;
        std::vector<std::vector<uint64_t>> cameras,heads,detections;
#ifdef MAPPED_TEST
#ifdef PLACE_TEST
        for(int n: {67,67,67,49,49,67,67,67,49,49}){
#else
        for(int n: {67,67,67,49,49}){
#endif
            std::string f=folder+"/case"+std::to_string(n);
            cameras.push_back(readhex(f+"/camera_packet.mem"));heads.push_back(readhex(f+"/head.mem"));detections.push_back(readhex(f+"/detection.mem"));
        }
#ifdef PLACE_TEST
        constexpr unsigned expected_results=10,expected_pixel=(58<<8)|43,expected_frames=11;
#else
        constexpr unsigned expected_results=5,expected_pixel=(58<<8)|43,expected_frames=5;
#endif
#else
        cameras.push_back(readhex(folder+"/camera_packet.mem"));heads.push_back(readhex(folder+"/head.mem"));detections.push_back(readhex(folder+"/detection.mem"));
        constexpr unsigned expected_results=1,expected_pixel=(16<<8)|66,expected_frames=5;
#endif
        Vconfirmed_grasp_system top;
        Pose observe={1500,1574,1833,648,1500,1300},approach={1500,1295,2225,1430,1500,1300};
        Pose descent={1500,1050,2280,1730,1500,1300},grip=descent,lift=approach;
#ifdef PRODUCTION_PROFILE
        auto source=readhex(folder+"/source_poses.mem");if(source.size()!=18)throw std::runtime_error("Missing current source pose fixture");
        for(int i=0;i<6;i++){observe[i]=source[i];approach[i]=source[i+6];descent[i]=source[i+12];}
        grip=descent;lift=approach;
#endif
        grip[5]=1700;lift[5]=1700;
        std::string expected,received; for(const auto& p:std::vector<Pose>{observe,approach,descent,grip,lift})expected+=packet(p);
#ifdef PLACE_TEST
        auto dest=readhex(folder+"/placement_poses.mem");if(dest.size()!=12)throw std::runtime_error("Missing destination pose fixture");
        Pose dest_approach,dest_descent;for(int i=0;i<6;i++){dest_approach[i]=dest[i];dest_descent[i]=dest[i+6];}
        Pose dest_a_closed=dest_approach,dest_d_closed=dest_descent,observe_closed=observe;
        dest_a_closed[5]=1700;dest_d_closed[5]=1700;observe_closed[5]=1700;
#ifdef SERIAL_BASE_CANDIDATE
        observe_closed[0]=lift[0];
#endif
        for(const auto& p:std::vector<Pose>{observe_closed,dest_a_closed,dest_d_closed,dest_descent,dest_approach,observe})expected+=packet(p);
#endif
        std::ofstream uart_trace("uart_commands.tsv");uart_trace<<"cycle\tid\tP\tT_ms\tgrouped\n";
        bool in_group=false;std::string command_text;std::vector<unsigned> received_base;
        uint64_t last_base_end=0,last_command_end=0;bool base_wait_pending=false;
        uint64_t start_frame=0,done_at=0,stop_at=0,last_result_at=0;unsigned nheads=0,results=0;
        bool previous_tx=true,rx_active=false;unsigned rx_timer=0,rx_bit=0,rx_byte=0;
        constexpr unsigned div=50000000/115200;
        for(uint64_t cycle=0;cycle<800000000;cycle++){
            top.rst_n=cycle>=8;top.enable=cycle>=200;top.capture_only=capture;
            constexpr unsigned stop_frames=expected_frames==11?7:2;
            if(stop && top.sent_frames==stop_frames && !stop_at)stop_at=cycle;
            top.estop=stop_at!=0;
            if(!start_frame && top.observe_ready && results<expected_results)start_frame=cycle+100;
            bool pin=true;
            if(start_frame && cycle>=start_frame && results<expected_results){uint64_t rel=cycle-start_frame;size_t byte=rel/540;unsigned bit=(rel%540)/54;
                if(byte<cameras[results].size()){uint8_t b=cameras[results][byte];pin=bit==0?false:bit==9?true:bool((b>>(bit-1))&1);}}
            top.camera_rx=pin;top.clk=0;top.eval();top.clk=1;top.eval();
            bool tx=top.servo_tx;
            if(rx_active){
                if(rx_timer==0){
                    if(rx_bit<8){rx_byte|=unsigned(tx)<<rx_bit;rx_bit++;rx_timer=div-1;}
                    else{if(!tx)throw std::runtime_error("Servo UART stop bit");received.push_back(char(rx_byte));rx_active=false;
                        if(rx_byte=='{' || rx_byte=='}')throw std::runtime_error("Grouped packet in sequential mode");
                        if(rx_byte=='{'){
#ifdef SERIAL_BASE_CANDIDATE
                            if(base_wait_pending && cycle-last_base_end<100000)throw std::runtime_error("Upper group issued before base movement wait");
#endif
                            base_wait_pending=false;in_group=true;
                        }
                        if(rx_byte=='}')in_group=false;
                        if(rx_byte=='#'){
                            if(last_command_end && cycle-last_command_end<100000)throw std::runtime_error("Next joint started before previous joint movement time");
                            last_command_end=cycle;command_text.clear();
                        }
                        if(!command_text.empty() || rx_byte=='#')command_text.push_back(char(rx_byte));
                        if(rx_byte=='!' && !command_text.empty()){
                            unsigned id=999,p=0,t=0;
                            if(sscanf(command_text.c_str(),"#%3uP%4uT%4u!",&id,&p,&t)!=3)throw std::runtime_error("Malformed command");
                            uart_trace<<cycle<<'\t'<<id<<'\t'<<p<<'\t'<<t<<'\t'<<in_group<<'\n';
                            if(id==0){
#ifdef SERIAL_BASE_CANDIDATE
                                if(in_group)throw std::runtime_error("Base incorrectly repeated in upper-axis group");
#endif
                                if(!in_group){received_base.push_back(p);last_base_end=cycle;base_wait_pending=true;}
                            }
                            last_command_end=cycle;command_text.clear();
                        }}
                }else rx_timer--;
            }else if(previous_tx&&!tx){rx_active=true;rx_timer=div+div/2-1;rx_bit=0;rx_byte=0;}
            previous_tx=tx;
            if(cycle<100199 && (top.accepted_cycles||top.sent_frames))throw std::runtime_error("Startup delay bypassed");
            if(top.fault && !stop_at)throw std::runtime_error("Integrated fault "+std::to_string(top.error_code));
            if(top.head_valid){
                unsigned index=nheads%360;
                if(results>=expected_results||index>=heads[results].size()||top.head_index!=index||uint32_t(top.head_acc)!=uint32_t(heads[results][index]))throw std::runtime_error("Real QNN head mismatch");nheads++;
            }
            if(top.model_result_valid){
                if(results>=expected_results||uint64_t(top.detections)!=detections[results][0])throw std::runtime_error("Detection mismatch");
                results++;last_result_at=cycle;start_frame=results<expected_results?cycle+200:0;
            }
#ifdef MAPPED_TEST
            if(results<expected_results && top.sent_frames>1)throw std::runtime_error("Motion before all target-selection and hand-clear results");
#endif
            if(top.state==6)throw std::runtime_error("Unexpected host-continue pause");
            if(top.cycle_done){
                if(done_at)throw std::runtime_error("Repeated grasp cycle");done_at=cycle;
                if(received!=expected)throw std::runtime_error("Six-joint sequence bytes mismatch: "+std::to_string(received.size()));
                if(results!=expected_results||nheads!=360*expected_results||top.accepted_cycles!=1||top.sent_frames!=expected_frames||top.completed_cycles!=1||top.result_frame_id!=37||top.target_pixel!=expected_pixel)throw std::runtime_error("Counts/frame/target mismatch");
#ifdef PLACE_TEST
                if(top.placement_pixel!=((40<<8)|35))throw std::runtime_error("Placement pixel mismatch");
#endif
            }
            if(stop_at && cycle>stop_at+100000){
                if(!top.fault||top.error_code!=2||!tx||top.sent_frames!=stop_frames||top.completed_cycles)throw std::runtime_error("Emergency stop did not lock sequence");
                std::cout<<"PASS: emergency_stop_blocks_remaining_packets\n";top.final();return 0;
            }
            if(capture && results==expected_results && cycle>last_result_at+1000000){
                if(top.fault||top.state!=4||top.sent_frames!=1||top.completed_cycles||!tx||received!=expected.substr(0,initial_packet_bytes))throw std::runtime_error("Capture-only mode allowed a grasp");
                std::cout<<"PASS: capture_only_keeps_real_camera_model_live_and_transmits_only_confirmed_observation\n";top.final();return 0;
            }
            if(done_at&&cycle>done_at+1000000){
#ifdef PLACE_TEST
                if(top.state!=4||top.selection_phase!=0)throw std::runtime_error("Must return to observation and wait for new pointing");
#else
                if(top.busy||top.state!=5)throw std::runtime_error("Must hold after pickup-only cycle");
#endif
                if(top.sent_frames!=expected_frames||top.accepted_cycles!=1||received.size()!=expected.size())throw std::runtime_error("Automatic repeat after completion");
                if(top.actuator_command_count!=expected_joint_commands || top.active_joint!=7)throw std::runtime_error("Serialized command count or idle joint mismatch");
                if(received_base!=expected_base)throw std::runtime_error("Actual base commands differ from expected ROM targets");
#ifdef SERIAL_BASE_CANDIDATE
                if(top.base_command_count!=expected_base.size() || top.base_command_p!=1500 || top.source_base_p!=approach[0] || top.destination_base_p!=dest_approach[0])throw std::runtime_error("Base debug path mismatch");
#endif
                std::cout<<"SERIAL_COMMANDS="<<expected_joint_commands<<" INITIAL_BYTES="<<initial_packet_bytes<<"\n";std::cout<<"BASE_P:";for(auto p:received_base)std::cout<<' '<<p;std::cout<<"\n";
                std::cout<<"PASS: actual_camera_UART real_FPGA_QNN_all_heads_exact continuous_stages="<<expected_frames<<" tx_bytes="<<received.size()<<" cycles="<<cycle<<"\n";
                top.final();return 0;
            }
        }
        throw std::runtime_error("Integrated pipeline timeout");
    }catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
