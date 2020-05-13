#include <string>
#include "math.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_server/image_loader.h"
#include "yaml-cpp/yaml.h"
#include "nav_msgs/GetMap.h"
#include <libgen.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "HttpRequest.h"
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
#include "test/stdSrv.h"
#include "test/movepos.h"
#include "test/status.h"
#include "test/gps_rcu.h"
#include "test/gps_tx2.h"
#include "test/plist.h"
#include "test/resume.h"
#include "test/suspend.h"
#include "test/laser.h"
#include <boost/network/protocol/http/client.hpp>
#include <nopoll.h>
#include <nopoll_private.h>
#include <pthread.h>
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/LaserScan.h>
//#include "test.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <fstream>
#include <chassis_control/chassis_sensor_data.h>
// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sys/stat.h>
//
#include <unistd.h>
#include <vector>
using namespace std;
using namespace boost::network;
using namespace boost::network::http;
using namespace cv;
using std::ofstream;


#define NAV_API_DEBUG   0 
//#define RECORD_PATH     0
unsigned int  RECORD_PATH  =   0;

const int g_iInternalMax = 1000;
const int iPathDivide = 100;
const std::string g_strWebSocketTimeInterval = "0.5";//push time for robot 
unsigned int uGridHeight =992; // heigth for map
unsigned int uGridWidth =992; // width for map
double fOriginX   =-24.8f;// x zero point for map
double fOriginY   =-24.8f;// y zero point for map
double fResolution=0.05000000074505806f;//ratio for map
const unsigned int g_uiCycleWebsocketTime = 500000;//us

char *modevar;
bool debugMode;
int RCU_exception_index=0;
/*============upload the config yaml for navigation ============*/
XmlRpc::XmlRpcValue destination_goal_tmp;
XmlRpc::XmlRpcValue planning_obstacles_tmp;
/*=============================================================*/
//char destination_goal_tmp[100];
int goal_compare_count;
int goal_compare_index[100];
int loop_count_tmp;

bool bRet = false;

std::string strGpsPos;
std::string strLoadMap;
std::string strWorkStatus;
std::string strPathContentGet1;
std::string strPathContentGet2;
std::string strIp ;
std::string strMapName;
std::string strMapListGet;
std::string strMapDataGet;
std::string strPathGet;
std::string strRealtimePosGet;
std::string strCurrentMapGet;
std::string strSuspendNavigation;
std::string strCancelNavigation;
std::string strResumeNavigation;
std::string strTaskExecute;
std::string laserstatus;
std::string strStaticInitial1;
std::string strStaticInitial2;
std::string savetask;
std::string strCancelstask;
std::string strSuspendstask;
std::string strResumestask;
std::string strCurrentinit;
std::string strSetspeedlevel;
std::string strPlanningbstacle;


//std::string strTaskArray  = "{\"name\":\"l604\",\"loop\":false,\"map_name\":\"\",\"tasks\":[{\"name\":\"PlayPathTask\",\"start_param\":{\"map_name\":\"l604\",\"path_name\":\"main_path\"}}]}";
std::string strTaskArray  = "{\"name\":\"l604\",\"loop\":true,\"loop_count\":100,\"map_name\":\"\",\"tasks\":[{\"name\":\"PlayPathTask\",\"start_param\":{\"map_name\":\"l604\",\"path_name\":\"main_path\"}}]}";
std::string strHomeTaskArray  = "{\"name\":\"\",\"loop\":false,\"loop_count\":1,\"map_name\":\"\",\"map_id\":\"\",\"tasks\":[{\"name\":\"NavigationTask\",\"start_param\":{\"map_name\":\"l604\",\"path_name\":\"main_path\",\"path_type\":0,\"position_name\":\"\"}}]}";
//std::string strGraphTaskArray  = "{\"name\":\"l604\",\"loop\":false,\"map_name\":\"\",\"tasks\":[{\"name\":\"PlayGraphPathTask\",\"start_param\":{\"map_name\":\"l604\",\"graph_name\":\"aaa\",\"graph_path_name\":\"main_path\"}}]}";
std::string strGraphTaskArray  = "{\"name\":\"l604\",\"loop\":true,\"loop_count\":100,\"map_name\":\"\",\"tasks\":[{\"name\":\"PlayGraphPathTask\",\"start_param\":{\"map_name\":\"l604\",\"graph_name\":\"aaa\",\"graph_path_name\":\"main_path\"}}]}";
std::string strRobotPos   = "{\"angle\":0, \"gridPosition\":{\"x\":0, \"y\":0},\"mapName\":\"abc\"}";
std::string strNavigationTask = "{\"name\":\"\",\"map_name\":\"\",\"map_id\":\"\",\"loop\":false,\"loop_count\":10,\"tasks\":[{\"name\":\"NavigationTask\",\"start_param\":{\"map_name\":\"l604\",\"path_name\":\"main_path\",\"path_type\":0,\"position_name\":\"\"}}]}";
std::string strNavigationGraphTask  = "{\"name\":\"l604\",\"loop\":true,\"loop_count\":100,\"map_name\":\"\",\"tasks\":[{\"name\":\"PlayGraphPathTask\",\"start_param\":{\"map_name\":\"l604\",\"graph_name\":\"aaa\",\"graph_path_name\":\"main_path\",\"position_name\":\"\"}}]}";
std::string strTask  = "{\"name\":\"map2\",\"loop\":false,\"loop_count\":10,\"map_name\":\"map2\",\"tasks\":[{\"name\":\"PlayPathTask\",\"start_param\":{\"map_name\":\"map2\",\"path_name\":\"main_path\"}}]}";
std::string str_init_current_position = "{\"mapName\":\"\",\"point\":{\"angle\":0,\"gridPosition\":{\"x\":0,\"y\":0}}}";
std::string str_obstacles = "{\"obstacles\":{\"circles\":[],\"lines\":[],\"polygons\":[],\"polylines\":[[{\"x\":0,\"y\":0},{\"x\":0,\"y\":0}]],\"rectangles\":[]}}";

HttpRequest* Http ;
char* str;
nav_msgs::GetMap::Response map_resp_1;
nav_msgs::GetMap::Response map_resp_2;
Json::Value  root;
Json::Value  root_task;// starttask req param
Json::Value root_1;
Json::Reader reader;
Json::FastWriter writer;

nopoll_bool debug = nopoll_false;    		    //websocket library variant
nopoll_bool show_critical_only = nopoll_false;	//websocket library variant

test::status sta;							    // global show robot current state
pthread_mutex_t g_mutex;						// lock for the variant of robot current state

std::string gStrIp;   						    //nav module ip
std::string gStrPort; 							//nav module port
std::string gStrStatePort; 						//nav module state port
std::string gStrUrl;  							//nav module url
std::string strHomeMapName;					    //home return map name
std::string strHomePathName;					//home return path name
/*========================define the param for gps=======================*/
float flatitude = 0.0f;
float flongitude= 0.0f;
float faltitude=0.0f;
int gps_status;
int navigation_model_status;
float real_latitude;
float real_longtitude;
float real_altitude;
float virtual_latitude;
float virtual_longtitude;
float virtual_altitude;
float transformLat_current;
float transformLon_current;
double a_length=6378140.0;        //long axis
double f_param=1/298.25722101;    //oblatense
double e_param = 0.00669438499959; //e^2
double pi = 3.14159265358979324;
test::gps_rcu gps_rcu_current;
test::gps_rcu gps_rcu_old;
float robot_speed;
float bearing ;
chassis_control::chassis_sensor_data chassis_sensor_data_current;
double origin_latitude;
double origin_longtitude;
double origin_bearing;
float origin_x;
float origin_y;
float origin_angle;
double pos_bearing;
double rad_tmp ;
double robot_destation;
double origin_shifting_angle;
bool   origin_switch;

/*========================================================================*/

/*=====================define the param for laser=========================*/
test::laser laser_info;
float laser_tmp_param[391];
float laser_distance_x;
float angle_increment = 0.00872664619237;
float three_m_deta;
bool main_path_index;
float main_path_deta;
/*=======================================================================*/




void __report_critical (noPollCtx * ctx, noPollDebugLevel level, const char * log_msg, noPollPtr user_data)
{
    if (level == NOPOLL_LEVEL_CRITICAL) 
    {
  	    printf ("CRITICAL: %s\n", log_msg);
	}
	return;
}
noPollCtx * create_ctx (void) {
	
	/* create a context */
	noPollCtx * ctx = nopoll_ctx_new ();
	nopoll_log_enable (ctx, debug);
	nopoll_log_color_enable (ctx, debug);

	/* configure handler */
	if (show_critical_only)
	    nopoll_log_set_handler (ctx, __report_critical, NULL);
	return ctx;
}
/*=====================get the robot status and update the status for RCU=========================*/
void* thread_GetRobotState (void*) {
	noPollCtx  * ctx;
	noPollConn * conn;
	Json::Value  root;
	Json::Reader reader;
	Json::FastWriter writer;
	http::client client;
	http::client::request request(strWorkStatus);
  	request<<header("Connection", "close");
  	http::client::response response;
	uint uTemp = 0;
	char         buffer[1024];
	int          bytes_read;
	noPollConnOpts * opts;
	std::string strTemp;
	std::string strWebsocketInterval =  "{\"device_interval\":"+ g_strWebSocketTimeInterval + "}";
	memset (buffer, 0, 1024);

	/* create context */
	ctx = create_ctx();

	/* check connections registered */
	if (nopoll_ctx_conns(ctx) != 0) {
		printf ("ERROR: expected to find 0 registered connections but found: %d\n", nopoll_ctx_conns (ctx));
		return (void*)1;
	} /* end if */

	nopoll_ctx_unref(ctx);

	/* reinit again */
	ctx = create_ctx ();

#if NAV_API_DEBUG 

	while(1)
	{
		pthread_mutex_lock(&g_mutex);
		sta.status = 1;
		pthread_mutex_unlock(&g_mutex);
		usleep(g_uiCycleWebsocketTime);
	}
#else

	while(1)
	{
		/* call to create a connection */
		conn= nopoll_conn_new(ctx, gStrIp.data(), gStrStatePort.data(), NULL,gStrUrl.data(),NULL,NULL);
		if (! nopoll_conn_is_ok(conn)) {
			ROS_ERROR("Web-Socket connect  error..\n");
			usleep(g_uiCycleWebsocketTime);
			continue;
		}

		if (! nopoll_conn_wait_until_connection_ready (conn, 15)) {		    
			ROS_ERROR("ERROR: send and receive is not ready..\n");
			usleep(g_uiCycleWebsocketTime);
			continue;
		}

		if (nopoll_conn_send_text(conn,strWebsocketInterval.data(),strlen(strWebsocketInterval.data())) != strlen(strWebsocketInterval.data()))
		{
			ROS_ERROR("set web-socket state sending time interval error... \n");	
			usleep(g_uiCycleWebsocketTime);
			continue;	
		}
		break;
	}

	while(1)
	{
		if (! nopoll_conn_is_ok(conn)) 
		{
			ROS_ERROR("Web-Socket connect  error..\n");
		  	conn= nopoll_conn_new(ctx, gStrIp.data(), gStrStatePort.data(), NULL,gStrUrl.data(),NULL,NULL);	
			if ((! nopoll_conn_is_ok(conn)) &&(! nopoll_conn_wait_until_connection_ready (conn, 15)) )
			{		    
				ROS_ERROR("ERROR: send and receive is not ready..\n");
				usleep(g_uiCycleWebsocketTime);
				continue;
			}
			usleep(g_uiCycleWebsocketTime);
			continue;		
		}

		bytes_read = nopoll_conn_read (conn, buffer, 2048, nopoll_true, 200);
		strTemp = buffer;
        int iTemp = strTemp.find("statusCode");
		if ((iTemp == -1) || (strTemp.length() < iTemp+15) )
		{
			usleep(g_uiCycleWebsocketTime);
			continue;
			
		}


		int iTemp2 = strTemp.find_first_of(",",iTemp);
		std::string strTemp1 = strTemp.substr(iTemp + 2+10, iTemp2 - iTemp -12);
//		printf("itemp= %d\n",iTemp);
//		printf("itemp2= %d\n",iTemp2);
//		printf("%s\n",strTemp1.data());
		pthread_mutex_lock(&g_mutex);
		if("0" == strTemp1)
		{
			sta.status = 0; //idle
		}
		else if("100" == strTemp1)
		{
			sta.status = 4; //pause
		}

 		else if("404" == strTemp1 || "408" == strTemp1 || "603" == strTemp1)
		{
			sta.status = 3; //goal point enreachabel;ureached;rotate failed
		}
		else if("303" == strTemp1|| "403" == strTemp1)
		{
			sta.status = 5; //waiting to avoid obstacle
		}
		else if("304" == strTemp1)
		{
			sta.status =6; //avoiding obstacle
		}
		else if("306" == strTemp1||"407" == strTemp1)
		{
			sta.status = 2; //follow path finished; reached
		}
		else if( strTemp1 == "302"||strTemp1 == "301"|| strTemp1 =="405" || strTemp1 == "406")
		{
			sta.status = 1; //following path; navigation yo start point
		}
		else if( strTemp1 == "1006")
		{
			sta.status = 7; //localization lost
		}

		pthread_mutex_unlock(&g_mutex);
		if(strTemp.find("follow path finished") != -1 ||strTemp.find("\"statusCode\":407") != -1)
		{
			sta.status = 2; 
		
	    }
		
		if(strTemp.find("robot stopped") != -1)
			sta.status = 0;
		if(debugMode)
		printf(" the robot all status is %s\n",buffer);
		//if (bytes_read != 19) {
		//	printf ("ERROR: expected to find 14 bytes but found %d..\n", bytes_read);
		//	return (void*)1;
		//} 
		usleep(g_uiCycleWebsocketTime);
	}
#endif
	/* finish connection */
	nopoll_conn_close (conn);
	
	/* finish */
	nopoll_ctx_unref (ctx);

	return (void*)1;
}
/*==================encode the base64 =========================*/
static std::string Encode(const unsigned char* Data,int DataByte)
{
 
    const char EncodeTable[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    std::string strEncode;
    unsigned char Tmp[4]={0};
    int LineLength=0;
    for(int i=0;i<(int)(DataByte / 3);i++)
    {
        Tmp[1] = *Data++;
        Tmp[2] = *Data++;
        Tmp[3] = *Data++;
        strEncode+= EncodeTable[Tmp[1] >> 2];
        strEncode+= EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
        strEncode+= EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
        strEncode+= EncodeTable[Tmp[3] & 0x3F];
        //if(LineLength+=4,LineLength==76) {strEncode+="\r\n";LineLength=0;}
    }

    int Mod=DataByte % 3;
    if(Mod==1)
    {
        Tmp[1] = *Data++;
        strEncode+= EncodeTable[(Tmp[1] & 0xFC) >> 2];
        strEncode+= EncodeTable[((Tmp[1] & 0x03) << 4)];
        strEncode+= "==";
    }
    else if(Mod==2)
    {
        Tmp[1] = *Data++;
        Tmp[2] = *Data++;
        strEncode+= EncodeTable[(Tmp[1] & 0xFC) >> 2];
        strEncode+= EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
        strEncode+= EncodeTable[((Tmp[2] & 0x0F) << 2)];
        strEncode+= "=";
    }
    
    return strEncode;
}
static bool mapCallback1(nav_msgs::GetMap::Request  &req,nav_msgs::GetMap::Response &res )
{	
	ros::NodeHandle private_nh("~");
	double origin[3];
    int negate;
    double occ_th, free_th;
    private_nh.param("negate", negate, 0);
    private_nh.param("occupied_thresh", occ_th, 0.65);
    private_nh.param("free_thresh", free_th, 0.196);
 	origin[0] = origin[1] = origin[2] = 0.0;

#if NAV_API_DEBUG 
    map_server::loadMapFromFile(&map_resp_1,"map1.png",fResolution,negate,occ_th,free_th, origin);
#else
	http::client client;	  
	http::client::request request(strMapDataGet+"map1");
	request << header("Connection", "close");
	http::client::response response = client.get(request);
	// std::cout << body(response) << std::endl;

	//std::ofstream fout("map1.png",std::ofstream::in|std::ofstream::out|std::ofstream::app);
	
	std::ofstream fout("//home//nvidia//.ros//map1.png");
	fout<<body(response);
	fout.close();

	map_server::loadMapFromFile(&map_resp_1,"map1.png",fResolution,negate,occ_th,free_th, origin);
#endif
	res = map_resp_1;
	//res.data = writer3.write(imgbase64);
	//res.bRet = true;
	return true;
}
nav_msgs::OccupancyGrid map1()
{	
	ros::NodeHandle private_nh("~");
	double origin[3];
    int negate;
    double occ_th, free_th;
    private_nh.param("negate", negate, 0);
    private_nh.param("occupied_thresh", occ_th, 0.65);
    private_nh.param("free_thresh", free_th, 0.196);

    origin[0] = origin[1] = origin[2] = 0.0;
	http::client client;	  
	http::client::request request(strMapDataGet+"map2");
	request << header("Connection", "close");
	http::client::response response = client.get(request);
	std::cout << body(response) << std::endl;

	std::ofstream fout("test.png",std::ofstream::out|std::ofstream::app);
	fout<<body(response);
	fout.close();

	map_server::loadMapFromFile(&map_resp_1,"test.png",fResolution,negate,occ_th,free_th, origin);
	
	return map_resp_1.map;
}
static bool mapCallback2(nav_msgs::GetMap::Request  &req,nav_msgs::GetMap::Response &res )
{
	ros::NodeHandle private_nh("~");
	double origin[3];
    int negate;
    double occ_th, free_th;
    private_nh.param("negate", negate, 0);
    private_nh.param("occupied_thresh", occ_th, 0.65);
    private_nh.param("free_thresh", free_th, 0.196);

    origin[0] = origin[1] = origin[2] = 0.0;

#if NAV_API_DEBUG 
	map_server::loadMapFromFile(&map_resp_2,"map2.png",fResolution,negate,occ_th,free_th, origin);
#else
	http::client client;	  
	http::client::request request(strMapDataGet+"map2");
	request << header("Connection", "close");
	http::client::response response = client.get(request);
	 // std::cout << body(response) << std::endl;

	std::ofstream fout("//home//nvidia//.ros//map2.png");
	fout<<body(response);
	fout.close();
	map_server::loadMapFromFile(&map_resp_2,"map2.png",fResolution,negate,occ_th,free_th, origin);
#endif
	res = map_resp_2;
	
	return true;
}
nav_msgs::OccupancyGrid map2()
{	
	ros::NodeHandle private_nh("~");
	double origin[3];
    int negate;
    double occ_th, free_th;
    private_nh.param("negate", negate, 0);
    private_nh.param("occupied_thresh", occ_th, 0.65);
    private_nh.param("free_thresh", free_th, 0.196);

    origin[0] = origin[1] = origin[2] = 0.0;
	http::client client;	  
	http::client::request request(strMapDataGet+"map2");
	request << header("Connection", "close");
	http::client::response response = client.get(request);
	 // std::cout << body(response) << std::endl;

	std::ofstream fout("map2.png",std::ofstream::out|std::ofstream::app);
	fout<<body(response);
	fout.close();

	map_server::loadMapFromFile(&map_resp_1,"map2.png",fResolution,negate,occ_th,free_th, origin);
	
	return map_resp_1.map;
}
static bool mapCallback3(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
#if NAV_API_DEBUG
	std::ifstream ifs("/home/data_sz/catkin_ws/src/test/src/maplist.json");
	if(!reader.parse(ifs,root))
	{
		ROS_ERROR("map list json parse failed\n");
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("map list get failed!\n");	
		return false;
	}
    res.data = writer.write(root);

	res.bRet = true;
#else
	http::client client;
	http::client::request request(strMapListGet);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("map list json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("map list HttpGet ask failed!\n");	
	    res.bRet = false;
		return false;
	}			
	

	res.data = body(response);
//std::cout<<res.data<<endl;
	res.bRet = true;
#endif
	return true;
}
bool maplist(test::plist&mapList)
{

	std::ifstream ifs("/home/nvidia/catkin_ws/src/test/src/example.json");
	if(!reader.parse(ifs,root))
	{
		ROS_INFO("json parse failed\n");
		return false;
	}

	else
	{//std::cout<<root.asString()<<endl;
		if(root["successed"].asBool())
		{

			if(0 == root["data"].size())
			{
				printf("there is no size for !\n");	
				return false;
			}
			mapList.header.stamp = ros::Time::now();
			for(int i =0 ; i<root["data"].size(); i++)
			{
				mapList.list.dim.push_back(std_msgs::MultiArrayDimension());
				mapList.list.dim[i].label = root["data"][i]["name"].asString();
				
				mapList.count++;
//printf("count is %d\n",mapList.count);
			}
		}
		else
		{	
			return false;
		}			
	}	
	
	return true;
}
static bool mapCallback4(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
#if NAV_API_DEBUG
	Json::Value  root1;
	Json::Reader reader1;	
	Json::FastWriter writer1;

	std::ifstream ifs("/home/nvidia/patrol_navi/src/test/src/pathlist.json");
	if(!reader.parse(ifs,root))
	{
		ROS_ERROR("path list json parse failed\n");
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("path list get failed!\n");	
		return false;
	}
    //res.data = writer.write(root);


	std::ifstream ifs1("/home/nvidia/patrol_navi/src/test/src/pathlistinfo.json");
	if(!reader.parse(ifs1,root1))
	{
		ROS_ERROR("path list info json parse failed\n");
		return false;
	}

	if(!root1["successed"].asBool())
	{
		ROS_ERROR("path list info  get failed!\n");	
		return false;
	}
    //res.data = writer.write(root);

	int iSize = root1["data"]["data"][0]["gridPosition"].size();
	int iTemp1 = iSize/g_iInternalMax;
	//std::cout<<"asdfds"<<endl;
	int iTemp2 = iTemp1*10;
	int iTemp3 = iTemp2*10;
	int iPathCount = iTemp1>=iPathDivide ? g_iInternalMax : ( iTemp2>=iPathDivide ? g_iInternalMax/10 : ( iTemp3>=iPathDivide ? g_iInternalMax/100 :1 ) );
	//std::cout<<"ipathcount is "<<iPathCount<<"iSize is "<<iSize<<endl;
	int j = 0;
	for(int i = 0 ; i < iSize ; i += iPathCount  )
	{
		root["data"][0]["gridPosition"][j] = root1["data"]["data"][0]["gridPosition"][i];
		j++;
	}
	
	root["data"][0]["gridPosition"][j] = root1["data"]["data"][0]["gridPosition"][iSize - 1];
	//root["data"][0]["gridPosition"] = root1["data"]["data"][0]["gridPosition"];

	res.data = writer1.write(root);
	std::cout<<res.data<<endl;
	res.bRet = true;
#else
    if ( RECORD_PATH )
	{
	Json::Value  root1;
	Json::Reader reader1;
	Json::FastWriter writer1;
	Json::Value  obstacle_param;
	Json::Reader obstacle_reader;
	Json::FastWriter obstacle_wirter;
	std::string strTemp;
	strMapName = req.data;
	//strMapName = "map1";
	strTemp = strPathGet + strMapName;

	http::client client;
	http::client::request request(strTemp);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);
	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("path list json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("path list HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}
	
	if(!obstacle_reader.parse(body(response),obstacle_param))
	{
		ROS_ERROR("path list json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!obstacle_param["successed"].asBool())
	{
		ROS_ERROR("path list HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}
	

	for(int path_index=0;path_index<root["data"].size();path_index++)
	{ 
	printf("the path count index is %d \n",path_index);
	strTemp = strPathContentGet1 + strMapName + strPathContentGet2 + root["data"][path_index]["name"].asString();
	if(root["data"][path_index]["name"].asString()=="main_path")
		{ main_path_index =true;
		}
	http::client::request request1(strTemp);
  	request1<<header("Connection", "close");
  	response= client.get(request1);

	if(!reader.parse(body(response),root1))
	{
		ROS_ERROR("path list full information json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root1["successed"].asBool())
	{
		ROS_ERROR("path list full information HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}			

	int iSize = root1["data"]["data"][0]["gridPosition"].size();
	printf("the gridposition is %d \n",iSize);
	int iTemp1 = iSize/g_iInternalMax;
	int iTemp2 = iSize*10/g_iInternalMax;
	int iTemp3 = iSize*100/g_iInternalMax;
	int iPathCount = iTemp1>=iPathDivide ? g_iInternalMax : ( iTemp2>=iPathDivide ? g_iInternalMax/10 : ( iTemp3>=iPathDivide ? g_iInternalMax/100 :1 ) );
	int j = 0;
	for(int i = 0 ; i < iSize ; i += iPathCount  )
	{
		root["data"][path_index]["gridPosition"][j]["x"] = root1["data"]["data"][0]["gridPosition"][i]["y"];
		root["data"][path_index]["gridPosition"][j]["y"] = root1["data"]["data"][0]["gridPosition"][i]["x"];
		obstacle_param["data"][path_index]["gridPosition"][j]["x"] = root1["data"]["data"][0]["gridPosition"][i]["x"];
		obstacle_param["data"][path_index]["gridPosition"][j]["y"] = root1["data"]["data"][0]["gridPosition"][i]["y"];		
		j++;

	}
	root["data"][path_index]["gridPosition"][j]["x"] = root1["data"]["data"][0]["gridPosition"][iSize - 1]["y"];
	root["data"][path_index]["gridPosition"][j]["y"] = root1["data"]["data"][0]["gridPosition"][iSize - 1]["x"];
    root["data"][path_index]["pointCount"]= j+1;
	obstacle_param["data"][path_index]["gridPosition"][j]["x"] = root1["data"]["data"][0]["gridPosition"][iSize - 1]["x"];
	obstacle_param["data"][path_index]["gridPosition"][j]["y"] = root1["data"]["data"][0]["gridPosition"][iSize - 1]["y"];
    obstacle_param["data"][path_index]["pointCount"]= j+1;	

	}
	res.data = writer1.write(root);
	std::string obstacle_list = obstacle_wirter.write(obstacle_param);
	std::string path_list_name ="/home/nvidia/"+strMapName+"_path_list.txt";
	std::ofstream fout(path_list_name);
	fout << obstacle_list;
	fout.close();
		
	if(debugMode)
	std::cout<<"the path data is"<<res.data<<endl;
	res.bRet = true;
	}
    else
	{

	Json::Value  root1;
	Json::Reader reader1;
	Json::FastWriter writer1;

	std::string strTemp;
	strMapName = req.data;
	//strMapName = "map1";
	strTemp =strIp + "/gs-robot/data/graph_paths?map_name=" + strMapName;


	std::ifstream ifs1("/home/nvidia/patrol_navi/src/test/src/pathlist.json");
	if(!reader.parse(ifs1,root1))
	{
		ROS_ERROR("path list info json parse failed\n");
		return false;
	}

	if(!root1["successed"].asBool())
	{
		ROS_ERROR("path list info  get failed!\n");	
		return false;
	}
    root1["data"][0]["mapName"]= strMapName;
	
	http::client client;
	http::client::request request(strTemp);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("path list json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("path list HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}			
	for(int path_index=0;path_index<root["data"].size();path_index++)
		{
	//root1["data"][0]["name"] = root["data"][0]["paths"][0]["name"];
	root1["data"][path_index]["name"] = root["data"][path_index]["name"];

	http::client::request request1(strTemp);
  	request1<<header("Connection", "close");
  	response= client.get(request1);


	int iSize = root["data"][path_index]["paths"][path_index]["points"].size();
	for(int i = 0 ; i < iSize ; i ++  )
	{
		root1["data"][path_index]["gridPosition"][i]["x"] = root["data"][path_index]["paths"][path_index]["points"][i]["gridPosition"]["y"];
		root1["data"][path_index]["gridPosition"][i]["y"] = root["data"][path_index]["paths"][path_index]["points"][i]["gridPosition"]["x"];
		
	}
    root1["data"][path_index]["pointCount"] = iSize;	

}
	//res.data = body(response);
	res.data = writer1.write(root);
	if(debugMode)
	std::cout<<res.data<<endl;
	res.bRet = true;
	}
    
#endif
	return true;
}
static bool mapCallback5(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
	Json::Value  root1;
	Json::Value  root2;
	Json::Reader reader1;
	Json::FastWriter writer1;
#if NAV_API_DEBUG
        std::string strRobotGpsPos   = "{\"longitude\":12, \"latitude\":23,\"orientation\":0}";
	res.bRet = true;
#else
	http::client client;
	http::client::request request(strGpsPos);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader1.parse(body(response),root1))
	{
		ROS_ERROR("get gps pos task  json parse failed\n");
		res.bRet = false;
		return false;
	}
	if(! (root1["msg"].asString() == "successed"))
	{
		ROS_ERROR("get gps pos task  HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}
        root2["longitude"]   = root1["data"]["longitude"]; 
        root2["latitude"]    = root1["data"]["latitude"]; 
        
	root2["orientation"] = 0; 
/*        root2["longitude"]   = flongitude; 
        root2["latitude"]    = flatitude; 
*/	
	res.data = writer1.write(root2);
	res.bRet = true;	
#endif
	return true;
}

void cal_virtual_gps(double bearing_tmp, double temp_rad, double E_D)
{		
		if(debugMode)
		{std::cout << "the bearing_tmp is "<<bearing_tmp <<"\n"<< std::endl;
		ROS_INFO("the origin_shifting_angle is %f",origin_shifting_angle);}
		if(bearing_tmp<=270 && bearing_tmp>=90)
				virtual_longtitude=origin_longtitude+(abs(cos(bearing_tmp*3.1416/180))*E_D*360)/(temp_rad*2*3.1416);
		if((bearing_tmp<=360 && bearing_tmp>270)||(bearing_tmp<90 && bearing_tmp>=0))	
				virtual_longtitude=origin_longtitude-(abs(cos(bearing_tmp*3.1416/180))*E_D*360)/(temp_rad*2*3.1416);
		if((bearing_tmp<=180&&bearing_tmp>=0))
				virtual_latitude = asin(abs(a_length*sin(origin_latitude*3.1416/180)+E_D*abs(sin(bearing_tmp*3.1416/180)))/a_length)*360/(2*3.1416);
		if(bearing_tmp<=360&&bearing_tmp>180)
				virtual_latitude = asin(abs(a_length*sin(origin_latitude*3.1416/180)-E_D*abs(sin(bearing_tmp*3.1416/180)))/a_length)*360/(2*3.1416);

}

bool robotpos(test::movepos&pos )
{

#if NAV_API_DEBUG
	pos.header.stamp = ros::Time::now();
	pos.x = 11;
	pos.y = 11;
	pos.theta = 11;
	pos.mapname =  "map1";
#else
	goal_compare_count=destination_goal_tmp.size();
	http::client client;
	http::client::request request1(strCurrentMapGet);
  	request1<<header("Connection", "close");
  	http::client::response response= client.get(request1);
 //std::cout << body(response) << std::endl;
	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("current map name json parse failed\n");
		return false;
	}
	
	pos.mapname =  root["currentMap"].asString();	

	http::client::request request(strRealtimePosGet);
  	request<<header("Connection", "close");
  	response= client.get(request);
//std::cout << body(response) << std::endl;
	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("current robot pose json parse failed\n");
		return false;
	}
	
	pos.x = root["gridPosition"]["y"].asFloat();
	pos.y = root["gridPosition"]["x"].asFloat();
//	if( (pos.theta >= -180 && pos.theta <= -90)|| (pos.theta>0 && pos.theta <=90 ) ) 
//	    pos.theta = root["angle"].asFloat();
//	else	
	pos.theta = 180 - root["angle"].asFloat();
	//bearing = origin_angle;
	rad_tmp = cos(origin_latitude*3.1416/180)*a_length;
	if(debugMode)
	ROS_INFO("real position is  %d,%d,%f \n",pos.x,pos.y,atan((pos.y-origin_y)/(pos.x-origin_x))*180/3.1416);
	robot_destation = sqrt((pos.y-origin_y)*(pos.y-origin_y)+(pos.x-origin_x)*(pos.x-origin_x))*0.05;
	if(pos.x>origin_x && pos.y>=origin_y)
		{pos_bearing=(origin_bearing+(atan((pos.y-origin_y)/(pos.x-origin_x))*180/3.1416-origin_shifting_angle))+90;
		if (pos_bearing<0)
			pos_bearing = pos_bearing+360;
		if (pos_bearing>360)
			pos_bearing = pos_bearing-360;
		cal_virtual_gps(pos_bearing,rad_tmp,robot_destation); //caculate the virtual gps
		}
	else if(pos.x>=origin_x && pos.y<origin_y)
		{pos_bearing=(origin_bearing+(atan((pos.y-origin_y)/(pos.x-origin_x))*180/3.1416-origin_shifting_angle))+90;
		if (pos_bearing<0)
			pos_bearing = pos_bearing+360;
		if (pos_bearing>360)
			pos_bearing = pos_bearing-360;
		cal_virtual_gps(pos_bearing,rad_tmp,robot_destation);//caculate the virtual gps
		}
	else if(pos.x<=origin_x && pos.y>origin_y)
		{pos_bearing=(origin_bearing+((180 + atan((pos.y-origin_y)/(pos.x-origin_x))*180/3.1416)-origin_shifting_angle))+90;
		if (pos_bearing<0)
			pos_bearing = pos_bearing+360;
		if (pos_bearing>360)
			pos_bearing = pos_bearing-360;
		cal_virtual_gps(pos_bearing,rad_tmp,robot_destation);//caculate the virtual gps
		}
	else if(pos.x<origin_x && pos.y<=origin_y)
		{pos_bearing=(origin_bearing+((180 + atan((pos.y-origin_y)/(pos.x-origin_x))*180/3.1416)-origin_shifting_angle))+90;
		if (pos_bearing<0)
			pos_bearing = pos_bearing+360;
		if (pos_bearing>360)
			pos_bearing = pos_bearing-360;
		cal_virtual_gps(pos_bearing,rad_tmp,robot_destation);//caculate the virtual gps
		}
		//pos.init_theta= root["angle"].asFloat();
			/*for(int tmp_index =0;tmp_index<goal_compare_count;tmp_index++)
		{ int x_tmp= int(destination_goal_tmp[tmp_index]["x"]);
			int y_tmp = int(destination_goal_tmp[tmp_index]["y"]);
			int angel_tmp = destination_goal_tmp[tmp_index]["angle"];
		if(((pos.x-x_tmp)^2+(pos.y-y_tmp)^2)<0.5&&(-5<(root["angle"].asFloat()-angel_tmp)<5))
		goal_compare_index[tmp_index]=1;
		else 
		goal_compare_index[tmp_index]=0;
		//std::cout << "the postion is \n" << goal_compare_index[tmp_index] <<endl;
		
		}*/
	
	pos.header.stamp = ros::Time::now();
#endif
	return true;
}
static bool mapCallback6(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
#if NAV_API_DEBUG
	res.bRet = true;
#else
	http::client client;
	http::client::request request(strCancelstask);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("cancel navi task json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("cancel navi task  HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}
		
	res.bRet = true;	
#endif
	return true;
}
static bool mapCallback7(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
#if NAV_API_DEBUG
res.bRet = true;
#else
	
	http::client client;
	http::client::request request(strSuspendNavigation);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("suspend navi task json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("suspend navi task HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}
		
	res.bRet = true;	
#endif
	return true;
}
static bool mapCallback8(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
#if NAV_API_DEBUG
	res.bRet = true;
#else

	http::client client;
	http::client::request request(strResumeNavigation);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("resume navi task  json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("resume navi task  HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}			
		
	res.bRet = true;	
#endif
	return true;
}
static bool mapCallback9(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
#if NAV_API_DEBUG
	res.bRet = true;
#else
	Json::Value  rootTemp;
	Json::Value  rootTemp1;
	Json::Value  init_current_position;
	Json::Value  path_list_param;
	Json::Value  path_list_param_post;
	Json::Reader path_list_param_post_reader;
	Json::Reader path_list_param_reader;
	//Json::Value  real_time_position;
	Json::Reader readerTemp;
	Json::Reader reader_init_current_position;
	Json::FastWriter writerTemp;
	Json::FastWriter path_list_param_writer;
	//Json::FastWriter writerTemp1;
	Json::FastWriter writer_init_position;
	Json::Reader readerTemp1;
	test::movepos current_pos;
	res.data = "";
	float x_1,y_1,x_2,y_2;
	float x1_1,x1_2,y1_1,y1_2;
	float x2_1,x2_2,y2_1,y2_2;
	float beta;
	float x1_tmp_left,y1_tmp_left,x1_tmp_right,y1_tmp_right,x2_tmp_left,y2_tmp_left,x2_tmp_right,y2_tmp_right;	

	
    if (RECORD_PATH)
	{
		if(!readerTemp.parse(strNavigationTask,rootTemp))
		{
			ROS_ERROR(" task execute info json parse failed\n");
			res.bRet = false;
			return false;
		}
	}
    else
	{
		if(!readerTemp.parse(strNavigationGraphTask,rootTemp))	
		{
			ROS_ERROR(" task execute info json parse failed\n");
			res.bRet = false;
			return false;
		}
	}
	if(!reader_init_current_position.parse(str_init_current_position,init_current_position))
		{
			ROS_ERROR(" init position info json parse failed\n");
			res.bRet = false;
			return false;
	}
	if(!reader.parse(req.data,root_task))
	{
		ROS_ERROR("task execute info json parse failed\n");
		res.bRet = false;
		return false;
	}
	/*========================== planning the obstacles============================*/
	/*std::string planning_obstacle_string = planning_obstacles_tmp["switch"];
	if(planning_obstacle_string =="on"){
	if(!path_list_param_post_reader.parse(str_obstacles,path_list_param_post))
		{
			ROS_ERROR(" post obstacle task execute info json parse failed\n");
			res.bRet = false;
			return false;
		}
	std::string path_data ="/home/nvidia/"+root_task["map_name"].asString()+"_path_list.txt"; 
	std::ifstream path_list_param_ifstr (path_data);
		if(!path_list_param_reader.parse(path_list_param_ifstr,path_list_param))
	{
		ROS_ERROR("obastacle path list info json parse failed\n");
		return false;
	}

	if(!path_list_param["successed"].asBool())
	{
		ROS_ERROR(" obastacle path list info  get failed!\n");	
		return false;
	}
		int d_left = int(int(planning_obstacles_tmp["left_gap"])/5);
		int d_right = int(int(planning_obstacles_tmp["right_gap"])/5);
		//std::cout <<"path_list_param is  "<< path_list_param<<"\n" <<std::endl;
		for(int i=0; i<path_list_param["data"].size();i++)
		{
		if(path_list_param["data"][i]["name"]==root_task["path_name"].asString())
			{ //printf("path name is found %s \n", path_list_param["data"][i]["name"]);
			
		for(int grid_count =0; (grid_count+3)<path_list_param["data"][i]["gridPosition"].size(); grid_count=grid_count+3)
		{ 
		//printf("the path_list_param size is %d \n", path_list_param["data"][i]["gridPosition"].size());
		//std::cout <<"data is  "<< path_list_param["data"][i]["gridPosition"][grid_count+3]["x"].asFloat()<<"\n" <<std::endl;
		x_1=path_list_param["data"][i]["gridPosition"][grid_count]["x"].asFloat();
		y_1=path_list_param["data"][i]["gridPosition"][grid_count]["y"].asFloat();
		x_2=path_list_param["data"][i]["gridPosition"][grid_count+3]["x"].asFloat();
		y_2=path_list_param["data"][i]["gridPosition"][grid_count+3]["y"].asFloat();
		if(debugMode)
		ROS_INFO(" the x1 y1 x2 y2 is %f %f %f %f \n",x_1,y_1,x_2,y_2);
		beta =float((x_2-x_1)/(y_2-y_1));
		if(y_2!=y_1){
		// caculate the equation for (x_1,y_1)
		x1_2=x_1-d_right/sqrt(1+beta*beta);
		x1_1=d_right/sqrt(1+beta*beta)+x_1;
		y1_2 = y_1 - beta*(x1_2-x_1);
		y1_1 = y_1 -beta*(x1_1-x_1);
		if(((y_2-y_1)*x1_2+(x_1-x_2)*y1_2+x_2*y_1-x_1*y_2)>0)//judge the position is right
			{x1_tmp_right=x1_2;
			y1_tmp_right=y1_2;}
		if(((y_2-y_1)*x1_1+(x_1-x_2)*y1_1+x_2*y_1-x_1*y_2)>0)//judge the position is right
			{x1_tmp_right=x1_1;
			y1_tmp_right=y1_1;}
		x1_2=x_1-d_left/sqrt(1+beta*beta);
		x1_1=d_left/sqrt(1+beta*beta)+x_1;		
		y1_2 = y_1 - beta*(x1_2-x_1);
		y1_1 = y_1 -beta*(x1_1-x_1);
		if(((y_2-y_1)*x1_2+(x_1-x_2)*y1_2+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x1_tmp_left=x1_2;
			y1_tmp_left=y1_2;}
		if(((y_2-y_1)*x1_1+(x_1-x_2)*y1_1+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x1_tmp_left=x1_1;
			y1_tmp_left=y1_1;}		
		// caculate the equation for (x_2,y_2)
		x2_2=x_2-d_right/sqrt(1+beta*beta);
		x2_1=d_right/sqrt(1+beta*beta)+x_2;
		y2_2 = y_2 - beta*(x2_2-x_2);
		y2_1 = y_2 -beta*(x2_1-x_2);
		if(((y_2-y_1)*x2_2+(x_1-x_2)*y2_2+x_2*y_1-x_1*y_2)>0)//judge the position is right
			{x2_tmp_right=x2_2;
			y2_tmp_right=y2_2;}
		if(((y_2-y_1)*x2_1+(x_1-x_2)*y2_1+x_2*y_1-x_1*y_2)>0)//judge the position is right
			{x2_tmp_right=x2_1;
			y2_tmp_right=y2_1;}
		x2_2=x_2-d_left/sqrt(1+beta*beta);
		x2_1=d_left/sqrt(1+beta*beta)+x_2;
		y2_2 = y_2 - beta*(x2_2-x_2);
		y2_1 = y_2 -beta*(x2_1-x_2);

		if(((y_2-y_1)*x2_2+(x_1-x_2)*y2_2+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x2_tmp_left=x2_2;
			y2_tmp_left=y2_2;}
		if(((y_2-y_1)*x2_1+(x_1-x_2)*y2_1+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x2_tmp_left=x2_1;
			y2_tmp_left=y2_1;}	

		}
		else if(y_2==y_1)
			{x1_2=x_1;
			x1_1=x_1;
			y1_2=y_1+d_right;
			y1_1=y_1-d_right;
			if(((y_2-y_1)*x1_2+(x_1-x_2)*y1_2+x_2*y_1-x_1*y_2)>0)//judge the position is right
				{x1_tmp_right=x1_2;
				y1_tmp_right=y1_2;}
			if(((y_2-y_1)*x1_1+(x_1-x_2)*y1_1+x_2*y_1-x_1*y_2)>0)//judge the position is right
				{x1_tmp_right=x1_1;
				y1_tmp_right=y1_1;}
			y1_2=y_1+d_left;
			y1_1=y_1-d_left;
			if(((y_2-y_1)*x1_2+(x_1-x_2)*y1_2+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x1_tmp_left=x1_2;
			y1_tmp_left=y1_2;}
			if(((y_2-y_1)*x1_1+(x_1-x_2)*y1_1+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x1_tmp_left=x1_1;
			y1_tmp_left=y1_1;}
			x2_2=x_2;
			x2_1=x_2;
			y2_2=y_2+d_right;
			y2_1=y_2-d_right;
			if(((y_2-y_1)*x2_2+(x_1-x_2)*y2_2+x_2*y_1-x_1*y_2)>0)//judge the position is right
				{x2_tmp_right=x2_2;
				y2_tmp_right=y2_2;}
			if(((y_2-y_1)*x2_1+(x_1-x_2)*y2_1+x_2*y_1-x_1*y_2)>0)//judge the position is right
				{x2_tmp_right=x1_1;
				y2_tmp_right=y1_1;}
			y2_2=y_2+d_left;
			y2_1=y_2-d_left;
			if(((y_2-y_1)*x2_2+(x_1-x_2)*y2_2+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x2_tmp_left=x2_2;
			y2_tmp_left=y2_2;}
			if(((y_2-y_1)*x2_1+(x_1-x_2)*y2_1+x_2*y_1-x_1*y_2)<0)//judge the position is left
			{x2_tmp_left=x2_1;
			y2_tmp_left=y2_1;}			
		
			}
		//ROS_INFO(" caculate obstacle ok\n");
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3][0]["y"] = int(y1_tmp_left);
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3][0]["x"] = int(x1_tmp_left);
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3][1]["y"] = int(y2_tmp_left);
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3][1]["x"] = int(x2_tmp_left);	
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3+1][0]["y"] = int(y1_tmp_right);
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3+1][0]["x"] = int(x1_tmp_right);
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3+1][1]["y"] = int(y2_tmp_right);
		path_list_param_post["obstacles"]["polylines"][2*grid_count/3+1][1]["x"] = int(x2_tmp_right);
			}
		}
		}
		ROS_INFO(" obstacle param  parse ok\n");
	std::string str_post_obastacle = path_list_param_writer.write(path_list_param_post);
	std::string str_obastacle_temp= strPlanningbstacle +root_task["map_name"].asString()+"&obstacle_name=obstacles";
	std::cout <<"the str_obastacle_temp is  "<< str_obastacle_temp<<"\n" <<std::endl;
	std::cout <<"the str_post_obastacle is  "<< str_post_obastacle<<"\n" <<std::endl;
	http::client client;
	http::client::request request_obastacle(str_obastacle_temp);
	request_obastacle<<header("Connection", "close");
	http::client::response response = client.post(request_obastacle,str_post_obastacle);
	ROS_INFO(" obstacle param  update ok\n");
	if(!path_list_param_post_reader.parse(body(response),path_list_param_post))
	{
		ROS_ERROR("update obstacle json parse failed\n");
		res.bRet = false;
		return false;
	
	}	
	std::cout << "update the obstacle"<< body(response) << std::endl;
	if(!(path_list_param_post["msg"].asString() == "successed"))
	{
		ROS_ERROR("update the obstacle  HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
		
	}
	
		}*/
	/*===========================end the planning obastacle============================*/
	/*=====================if not planning obastacle==========================*/ 
	 /*else if(planning_obstacle_string =="off")
		{
		std::string str_obastacle_temp= strPlanningbstacle +root_task["map_name"].asString()+"&obstacle_name=obstacles";
		http::client client;
		http::client::request request_obastacle(str_obastacle_temp);
		request_obastacle<<header("Connection", "close");
		
		http::client::response response = client.post(request_obastacle,str_obstacles);
		if(!path_list_param_post_reader.parse(body(response),path_list_param_post))
		{
		ROS_ERROR("update obstacle json parse failed\n");
		res.bRet = false;
		return false;
	
		}	
		std::cout << "update the obstacle"<< body(response) << std::endl;
		if(!(path_list_param_post["msg"].asString() == "successed"))
		{
		ROS_ERROR("update the obstacle  HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
		}
		}*/
	//ROS_INFO(" task json parse ok\n");
	//ROS_INFO(" %d \n",rootTemp["tasks"].size());
	rootTemp["map_name"] = root_task["map_name"].asString();
	rootTemp["loop"] = true;
	rootTemp["loop_count"]=int(loop_count_tmp);
	ROS_INFO(" the number of path is %d \n",destination_goal_tmp.size());
	
	// get the realtime position
	http::client client1;
	http::client::request request_realtime(strRealtimePosGet);
  	request_realtime<<header("Connection", "close");
  	http::client::response response_realtime= client1.get(request_realtime);
		if(!reader.parse(body(response_realtime),root_1))
	{
		ROS_ERROR("current position json parse failed\n");
		return false;
	}
		current_pos.x = root_1["gridPosition"]["x"].asFloat();
		current_pos.y = root_1["gridPosition"]["y"].asFloat();
		current_pos.theta = root_1["angle"].asFloat();
	std::cout << "realtime postion is " << current_pos <<endl;	
	// caculate the pass goal
	int path_count_temp;
	for(int path_index=0;path_index<destination_goal_tmp.size();path_index++)
		{if (destination_goal_tmp[path_index]["path_name"]==root_task["path_name"].asString())
		path_count_temp=path_index;
		}
	for (int count_temp=0;count_temp<destination_goal_tmp[path_count_temp]["position_name"].size();count_temp++){
    if( RECORD_PATH )
		
		{rootTemp["tasks"][count_temp]["name"]="NavigationTask";
		 rootTemp["tasks"][count_temp]["start_param"]["map_name"] = root_task["map_name"].asString();
		 rootTemp["tasks"][count_temp]["start_param"]["path_name"] = root_task["path_name"].asString();
		 rootTemp["tasks"][count_temp]["start_param"]["path_type"] = 0;
		 rootTemp["tasks"][count_temp]["start_param"]["position_name"] = std::string(destination_goal_tmp[path_count_temp]["position_name"][count_temp]);
		 //rootTemp["tasks"][count_temp]["start_param"]["destination"]["gridPosition"]["x"] = int(destination_goal_tmp[count_temp]["x"]);
		// rootTemp["tasks"][count_temp]["start_param"]["destination"]["gridPosition"]["y"] = int(destination_goal_tmp[count_temp]["y"]);
		 
	
	ROS_INFO(" task json get successfully\n");}
    else
	{

	rootTemp["name"] = root_task["map_name"].asString();
    rootTemp["tasks"][0]["start_param"]["map_name"] = rootTemp["map_name"];
	//rootTemp["tasks"][0]["start_param"]["graph_name"] =  root["path_name"].asString();
	//rootTemp["tasks"][0]["start_param"]["graph_path_name"] = root["path_name"].asString()+ "_path0";
	}
	}
	
	// get current postion

	//std::cout << init_current_position << std::endl;
	std::string strTemp = writerTemp.write(rootTemp);
	std::string strTemp1= strStaticInitial1 + rootTemp["map_name"].asString() + strStaticInitial2;

	//printf("%s\n",strTemp.data());
	std::cout << "strtemp data is \n" << strTemp.data() <<endl;
	/*===================get the current map information======================*/
	/*http::client::request request2(strLoadMap + rootTemp["map_name"].asString());
  	request2<<header("Connection", "close");
  	http::client::response response2= client1.get(request2);

	if(!reader.parse(body(response2),root))
	{
		ROS_ERROR("navi laod map  execute json parse failed\n");
		res.bRet = false;
		return false;
	}

	 std::cout << body(response2) << std::endl;
	if(!root["successed"].asBool())
	{
		ROS_ERROR("navi load map execute HttpGet ask failed!\n");
		res.bRet = false;
		return false;
	}*/
	//robotpos(current_pos);
	//printf ("current pos is %d %d %d ",current_pos.theta,current_pos.x,current_pos.y);

	init_current_position["mapName"]=rootTemp["map_name"].asString();
	init_current_position["point"]["angle"]=int(current_pos.theta);
	init_current_position["point"]["gridPosition"]["x"]=int(current_pos.x);
	init_current_position["point"]["gridPosition"]["y"]=int(current_pos.y);
	std::string str_init_postion=writer_init_position.write(init_current_position);
	std::cout << "init_position is " << str_init_postion.data()<<std::endl;
/*=====================init the position==============================*/
	/*http::client::request request1(strCurrentinit);
  	request1<<header("Connection", "close");
  	http::client::response response1= client1.post(request1,str_init_postion);
	if(!reader.parse(body(response1),root))
	{
		ROS_ERROR("current initial execute json parse failed\n");
		res.bRet = false;
		return false;
	}

	 std::cout << body(response1) << std::endl;
	if(!root["successed"].asBool())
	{
		ROS_ERROR("current initial execute HttpGet ask failed!\n");
		res.bRet = false;
		return false;
	}*/			
//	http::client client;
	http::client::request request(strTaskExecute);
  	request<<header("Connection", "close");
  	http::client::response response_task= client1.post(request,strTemp);

	if(!reader.parse(body(response_task),root))
	{
		ROS_ERROR("navi execute json parse failed\n");
		res.bRet = false;
		return false;
	}

	 std::cout << body(response_task) << std::endl;
	if(!root["successed"].asBool())
	{
		ROS_ERROR("navi execute HttpGet ask failed!\n");
		res.bRet = false;
		return false;
	}			

	res.bRet = true;
#endif
	return true;
}

bool robotstatus(test::status sta)
{
#ifndef NAV_API_DEBUG
	http::client client;
	http::client::request request(strNavStateQuery);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_INFO(" navigation json parse failed\n");		
		return false;
	}
	else
	{
		if(root["successed"].asBool())
		{
			printf("navigation stats HttpGet request succesful !\n");	
			}
		else
		{				
			return false;
		}			
	}	

	sta.status = root["data"]["work_status"]["work_type"].asString();	

#else
	sta.status = sta.ACTIVE;
#endif	
	return true;
}
static bool mapCallback10(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
#if NAV_API_DEBUG
	res.bRet = true;
#else
	Json::Value  rootTemp;
	Json::Reader readerTemp;
	Json::FastWriter writerTemp;
	res.data = "";

	http::client client;
  	http::client::response response;
	/*http::client::request request(strWorkStatus);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("work status json parse failed\n");
		res.bRet = false;
		return false;
	}
	if(!root["successed"].asBool())
	{
		ROS_ERROR("work status HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}*/
	printf("mutex\n");
	//pthread_mutex_lock(&g_mutex);
	if( (sta.status == 1 || sta.status ==5 || sta.status ==6) )
    {
		ROS_ERROR("robot is working , can not go home now!\n");	
		res.data ="Error : robot is working , can not go home now!";
		res.bRet = false;
		return false;
	}
	//pthread_mutex_unlock(&g_mutex);

	std::string strTemp1= strStaticInitial1 + strHomeMapName + strStaticInitial2;     
	int path_count_temp;
	for(int path_index=0;path_index<destination_goal_tmp.size();path_index++)
		{if (destination_goal_tmp[path_index]["path_name"]==root_task["path_name"].asString())
		path_count_temp=path_index;
		}
	
	// 初始化代码，新版本注释掉
	/*http::client::request request2(strLoadMap + strHomeMapName);
  	request2<<header("Connection", "close");
  	http::client::response response2= client.get(request2);

	if(!reader.parse(body(response2),root))
	{
		ROS_ERROR("home laod map  execute json parse failed\n");
		res.bRet = false;
		return false;
	}

	 std::cout << body(response2) << std::endl;
	if(!root["successed"].asBool())
	{
		ROS_ERROR("home load map execute HttpGet ask failed!\n");
		res.bRet = false;
		return false;
	}

	http::client::request request1(strTemp1);
  	request1<<header("Connection", "close");
  	http::client::response response1= client.get(request1);
	if(!reader.parse(body(response1),root))
	{
		ROS_ERROR("home origin initial execute json parse failed\n");
		res.bRet = false;
		return false;
	}

	 std::cout << body(response1) << std::endl;
	if(!root["successed"].asBool())
	{
		ROS_ERROR("home origin initial execute HttpGet ask failed!\n");
		res.bRet = false;
		return false;
	}*/
	/*	if(!reader.parse(req.data,root))
	{
		ROS_ERROR("task execute info json parse failed\n");
		res.bRet = false;
		return false;
	}*/
    if(RECORD_PATH)
	{

		if(!readerTemp.parse(strHomeTaskArray,rootTemp))
		{
			ROS_ERROR("home json parse failed\n");
			res.bRet = false;
			return false;
		}
		std::cout << root_task << std::endl;
		rootTemp["map_name"] = root_task["map_name"].asString();
		rootTemp["tasks"][0]["start_param"]["map_name"] = root_task["map_name"].asString();
		rootTemp["tasks"][0]["start_param"]["path_name"] = root_task["path_name"].asString();
		rootTemp["tasks"][0]["start_param"]["path_type"] = 0;
		rootTemp["tasks"][0]["start_param"]["position_name"] = std::string(destination_goal_tmp[path_count_temp]["position_name"][0]);
		printf("test\n");
	}
    else
	{

		if(!readerTemp.parse(strGraphTaskArray,rootTemp))
		{
			ROS_ERROR("home json parse failed\n");
			res.bRet = false;
			return false;
		}

		rootTemp["map_name"] = root["map_name"].asString();
		rootTemp["tasks"][0]["start_param"]["map_name"] = root["map_name"].asString();
		rootTemp["tasks"][0]["start_param"]["path_name"] = root["path_name"].asString();
		rootTemp["tasks"][0]["start_param"]["path_type"] = 0;
		rootTemp["tasks"][0]["start_param"]["position_name"] = std::string(destination_goal_tmp[path_count_temp]["position_name"][0]);

	}

	std::string strTemp = writerTemp.write(rootTemp);

	
	http::client::request request3(strTaskExecute);
  	request3<<header("Connection", "close");
  	response= client.post(request3,strTemp);

	 std::cout << body(response) << std::endl;
	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("home  retunr json parse failed\n");
		res.bRet = false;
		return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("home return execute HttpGet ask failed!\n");	
		res.bRet = false;
		return false;
	}			
	
	res.bRet = true;
#endif
	return true;
}

static bool mapCallback11(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
	Json::FastWriter writer11;
	Json::Value map1_mesg;
        //cv::Mat cv_img;
		
        //std::string filename = "map1.png";
        //int cv_read_flag = (CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat cv_img_origin = cv::imread("/home/nvidia/.ros/map1.png",CV_LOAD_IMAGE_UNCHANGED);
        if (!cv_img_origin.data) {
                std::cout << "Could not open or find the map1 file :" <<endl;
                //cv_img = cv_img_origin;
        }
	
        int height = cv_img_origin.rows;
        int width = cv_img_origin.cols;
		cv::Mat img_1(width,height,cv_img_origin.type());
		cv::Mat img_1_1(width,height,cv_img_origin.type());
		//cv::Mat img_1
		if(debugMode)
        ROS_INFO("map_1 width and height is %d %d ",cv_img_origin.cols,cv_img_origin.rows);
		std::cout << "map1 type is " << cv_img_origin.size() <<endl;
		std::cout << "img1 type is " << img_1.size() <<endl;
		cv::transpose(cv_img_origin,img_1);
		cv::flip(img_1,img_1_1,1);
		cv::imwrite("/home/nvidia/.ros/map1_1.png",img_1_1);
        //cv::resize(cv_img_origin,cv_img,cv::Size(cv_img_origin.cols,cv_img_origin.rows));
        //cv::imwrite("~/.ros/map1_1.png",cv_img);
		//cv_img_origin.row(0).copyTo(img_1.col(0));	
		std::vector<uchar> vecImg;                               //Mat image vector<uchar>
		std::vector<int> vecCompression_params;
		vecCompression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		vecCompression_params.push_back(2);
		imencode(".png", img_1_1, vecImg,vecCompression_params);
		//imencode(".png", cv_img_origin, vecImg, vecCompression_params);
		std::string imgbase64 = Encode(vecImg.data(), vecImg.size());
		if(debugMode)
		ROS_INFO("s%",imgbase64);
		//std::string imgBase64_1 = Encode(buffer, size);
		//std::string encoded = Encode((unsigned char*)buffer, lSize);
		map1_mesg["map"]="map1";
		map1_mesg["base64"]=imgbase64;
        res.data = writer11.write(map1_mesg);
        res.bRet = true;
		//f.close();		
	return true;
}

static bool mapCallback12(test::stdSrv::Request&req,test::stdSrv::Response&res )
{
	Json::FastWriter writer12;
	Json::Value map2_mesg;
	
        //cv::Mat cv_img_2;
		//cv::Mat img_2;
        //std::string filename = "map1.png";
        //int cv_read_flag = (CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat cv_img_origin_2 = cv::imread("/home/nvidia/.ros/map2.png",CV_LOAD_IMAGE_UNCHANGED);
        if (!cv_img_origin_2.data) {
                std::cout << "Could not open or find the map2 file :" <<endl;
                //cv_img_2 = cv_img_origin_2;
        }

        int height = cv_img_origin_2.rows;
        int width = cv_img_origin_2.cols;
		cv::Mat img_2(width,height,cv_img_origin_2.type());
		cv::Mat img_2_2(width,height,cv_img_origin_2.type());
		cv::transpose(cv_img_origin_2,img_2);
		cv::flip(img_2,img_2_2,1);
		if(debugMode)
        ROS_INFO("map_2 width length %d %d",cv_img_origin_2.cols,cv_img_origin_2.rows);
        //cv::resize(cv_img_origin_2,cv_img_2,cv::Size(cv_img_origin_2.cols,cv_img_origin_2.rows));
        //cv::imwrite("~/.ros/map2_1.png",cv_img);
			
		std::vector<uchar> vecImg_2;                               //Mat image vector<uchar>
		std::vector<int> vecCompression_params_2;
		vecCompression_params_2.push_back(CV_IMWRITE_PNG_COMPRESSION);
		vecCompression_params_2.push_back(2);
		imencode(".png", img_2_2, vecImg_2,vecCompression_params_2);
		//imencode(".png", cv_img_origin_2, vecImg_2, vecCompression_params_2);
		std::string imgbase64_1 = Encode(vecImg_2.data(), vecImg_2.size());
		if(debugMode)
		ROS_INFO("s%",imgbase64_1);
		map2_mesg["map"]="map2";
		map2_mesg["base64"]=imgbase64_1;
        res.data = writer12.write(map2_mesg);
        res.bRet = true;
		//f.close();
	
	return true;
}
static bool init_the_map_callback(test::stdSrv::Request&req,test::stdSrv::Response&res)
{	
	if(!reader.parse(req.data,root_task))
	{
		ROS_ERROR("task execute info json parse failed\n");
		res.bRet = false;
		return false;
	}
	/*==================load  the current map=====================*/
	http::client client1;
	http::client::request request2(strLoadMap + root_task["map_name"].asString());
  	request2<<header("Connection", "close");
  	http::client::response response2= client1.get(request2);

	if(!reader.parse(body(response2),root))
	{
		ROS_ERROR("navi laod map  execute json parse failed\n");
		res.bRet = false;
		return false;
	}

	 std::cout << body(response2) << std::endl;
	if(!root["successed"].asBool())
	{
		ROS_ERROR("navi load map execute HttpGet ask failed!\n");
		res.bRet = false;
		return false;
	}
	/*==================init the origin position =====================*/
	std::string str_origin_postion= strStaticInitial1 + root_task["map_name"].asString() + strStaticInitial2;
	http::client::request request1(str_origin_postion);
  	request1<<header("Connection", "close");
	http::client::response response1= client1.get(request1);
	if(!reader.parse(body(response1),root))
	{
		ROS_ERROR("Origin initial execute json parse failed\n");
		res.bRet = false;
		return false;
	}

	 std::cout << body(response1) << std::endl;
	if(!root["successed"].asBool())
	{
		ROS_ERROR("Origin initial execute HttpGet ask failed!\n");
		res.bRet = false;
		return false;
	}
	/*==================get the origin position =====================*/
	http::client::request request(strRealtimePosGet);
  	request<<header("Connection", "close");
  	http::client::response response= client1.get(request);
	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("get the current robot pose json parse failed\n");
		return false;
	}
	origin_x = root["gridPosition"]["x"].asFloat();
	origin_y = root["gridPosition"]["y"].asFloat();
	origin_angle = root["angle"].asFloat();
	origin_latitude = root_task["origin_latitude"].asFloat();
	origin_longtitude = root_task["origin_longtitude"].asFloat();
	origin_bearing = root_task["origin_bearing"].asFloat();

}
static inline bool is_base64(const char c)
{
    return (isalnum(c) || (c == '+') || (c == '/'));
}
//caculate the latitude
double calLat(double B, double L,double H) {
		double x_angle=B*pi/180;
		double y_angle=L*pi/180;
		if(debugMode)
		printf("the x_angle and y_angle is %f %f\n",x_angle,y_angle);
		double N = a_length/sqrt(1-e_param*sin(x_angle)*sin(x_angle));
		double resultLat = (N+H)*cos(x_angle)*cos(y_angle);
        
        return resultLat;
}

//caculate the longtitude
double calLon(double B, double L,double H) {
		double x_angle=B*pi/180;
		double y_angle=L*pi/180;
		double N = a_length/sqrt(1-e_param*sin(x_angle)*sin(x_angle));		
		double resultLon =(N+H)*cos(x_angle)*sin(y_angle);
        return resultLon;
}
double calAl(double B, double L,double H) {
		double x_angle=B*pi/180;
		double y_angle=L*pi/180;
		double N = a_length/sqrt(1-e_param*sin(x_angle)*sin(x_angle));		
		double resultLon =(N*(1-e_param)+H)*sin(x_angle);
        return resultLon;
}


void suspend_callback(const test::suspend::ConstPtr& suspend_cmd)
{	int suspend_index = suspend_cmd->suspend_cmd;
	ROS_INFO("suspend_index = %d",suspend_index);
	ROS_INFO("sta.status = %d",sta.status);
	std::string suspend_speed = strSetspeedlevel +"0";
	if (sta.status == 1 || sta.status == 5 || sta.status == 6)
	{
	if (suspend_index ==1)
	ROS_INFO("robot is to be moderated");
	
	http::client client;
	http::client::request request(suspend_speed);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("suspend navi task json parse failed\n");
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("suspend navi task HttpGet ask failed!\n");	
		
	}
	RCU_exception_index=1;
	}
}
void resume_callback(const test::resume::ConstPtr& resume_cmd)

{	int resume_index = resume_cmd->resume_cmd;
	ROS_INFO("resume_index = %d",resume_index);
	ROS_INFO("sta.status = %d",sta.status);
	std::string resume_speed = strSetspeedlevel +"2";
	if (sta.status == 4 || RCU_exception_index==1)
	{
	if (resume_index ==1)
	ROS_INFO("robot is to be resumed speed ");	
	http::client client;
	http::client::request request(resume_speed);
  	request<<header("Connection", "close");
  	http::client::response response= client.get(request);

	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("resume navi task  json parse failed\n");
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR("resume navi task  HttpGet ask failed!\n");	
	}
	RCU_exception_index=0;
	}			
}

void dataCallBack(const sensor_msgs::NavSatFix& msg)
{
	if(msg.latitude < 0.001 || msg.longitude < 0.001)
		return;
	flatitude = msg.latitude;
	flongitude= msg.longitude;
	if(debugMode)
	ROS_INFO("%f %f\n",msg.latitude,msg.longitude);
}
void GTK_gps_callback( const sensor_msgs::NavSatFix& msg)
{ 	
	/*if(msg.latitude < 0.001 || msg.longitude < 0.001)
		return;*/
	gps_status = msg.status.status;
	flatitude = msg.latitude;
	flongitude= msg.longitude;
	faltitude = msg.altitude;
	if(debugMode)
	ROS_INFO("the latitude and the longtitude get from the raw_gps is %f %f\n",msg.latitude,msg.longitude);

}
void gps_fusion(const test::gps_rcu& msg_rcu_gps)
{
	 if(msg_rcu_gps.rcu_latitude)
	{
		gps_rcu_current.rcu_latitude = msg_rcu_gps.rcu_latitude;
		gps_rcu_current.rcu_longtitude = msg_rcu_gps.rcu_longtitude;
		gps_rcu_current.rcu_altitude = msg_rcu_gps.rcu_altitude;
		gps_rcu_current.rcu_bearing = msg_rcu_gps.rcu_bearing;
		if(debugMode)
		ROS_INFO("the latitude and the longtitude get from the rcu_gps is %f %f\n",gps_rcu_current.rcu_latitude,gps_rcu_current.rcu_longtitude);
	}
	else {
			gps_rcu_current.rcu_latitude =0;
			gps_rcu_current.rcu_longtitude =0;
			gps_rcu_current.rcu_altitude = 0;
			gps_rcu_current.rcu_bearing =0;
		}


	bearing= gps_rcu_current.rcu_bearing;
	bearing = bearing+180;
	if(debugMode)
	{ROS_INFO("the orientation is %f \n",bearing);
	ROS_INFO("the rcu_bearing is %f \n",gps_rcu_current.rcu_bearing);
	
	
	ROS_INFO("the latitude and the longtitude from RCU is %f %f \n",gps_rcu_current.rcu_longtitude,gps_rcu_current.rcu_latitude);
	}
	if(bRet==true&&origin_switch==true)
		{
			real_latitude= virtual_latitude;
			real_longtitude= virtual_longtitude;
			real_altitude = gps_rcu_current.rcu_altitude;
		}
	else if (origin_switch==false)
		{
	if(gps_rcu_current.rcu_longtitude)
		{
		if(gps_rcu_old.rcu_latitude==0||gps_rcu_old.rcu_longtitude==0)
			{gps_rcu_old.rcu_latitude = gps_rcu_current.rcu_latitude;
			gps_rcu_old.rcu_longtitude = gps_rcu_current.rcu_longtitude;
			real_longtitude= gps_rcu_current.rcu_longtitude;
			real_latitude= gps_rcu_current.rcu_latitude;
			}
		else if(gps_rcu_old.rcu_latitude != gps_rcu_current.rcu_latitude || gps_rcu_old.rcu_longtitude != gps_rcu_current.rcu_longtitude)
			{real_latitude= gps_rcu_current.rcu_latitude;
			real_longtitude= gps_rcu_current.rcu_longtitude;
			real_altitude = gps_rcu_current.rcu_altitude;
			}
		else if(gps_rcu_old.rcu_latitude == gps_rcu_current.rcu_latitude || gps_rcu_old.rcu_longtitude == gps_rcu_current.rcu_longtitude)
			{if(debugMode)
			ROS_INFO("start caculate the GPS by sensor \n");
			if (bearing>360)
				bearing =bearing-360;
			float r_tmp = cos(gps_rcu_old.rcu_latitude*3.1416/180)*a_length;
			if(bearing<270 && bearing>90)
				real_longtitude=real_longtitude+(abs(cos(bearing*3.1416/180))*4*robot_speed*360)/(r_tmp*2*3.1416);
			if((bearing<=360 && bearing>=270)||(bearing<=90 && bearing>=0))	
				real_longtitude=real_longtitude-(abs(cos(bearing*3.1416/180))*4*robot_speed*360)/(r_tmp*2*3.1416);
			if(bearing<=180&&bearing>=0)
				real_latitude = asin(abs(a_length*sin(gps_rcu_old.rcu_latitude*3.1416/180)+4*robot_speed*sin(bearing*3.1416/180))/a_length)*360/(2*3.1416);
			if(bearing<=360&&bearing>=180)
				real_latitude = asin(abs(a_length*sin(gps_rcu_old.rcu_latitude*3.1416/180)-4*robot_speed*sin(bearing*3.1416/180))/a_length)*360/(2*3.1416);
			real_altitude = gps_rcu_current.rcu_altitude;
			}
			
				
		{gps_rcu_old.rcu_latitude = gps_rcu_current.rcu_latitude;
		gps_rcu_old.rcu_longtitude = gps_rcu_current.rcu_longtitude;
		if(debugMode){
		ROS_INFO("the old_latitude is %f \n",gps_rcu_old.rcu_latitude);
		ROS_INFO("the old_longtitude is %f \n",gps_rcu_old.rcu_longtitude);}
		}
	}
	else 
		{real_latitude=0;
		real_longtitude=0;
		real_altitude=0;
		}
		}
	if(real_longtitude!=0&&real_latitude!=0&&real_altitude!=0)
		{
	transformLat_current = calLat(real_longtitude,real_latitude,real_altitude);
	transformLon_current = calLon(real_longtitude,real_latitude,real_altitude);
	if(debugMode)
	printf("the transformLat_current and transformLon_current is %f %f \n",transformLat_current,transformLon_current);
		}

}
void check_map_diff(test::plist &plist_param)
{
	Json::Value map_list;
	Json::Reader map_list_reader;
	Json::FastWriter map_list_writer;
	http::client check_map;
	http::client::request request(strMapListGet);
	request << header("Connection", "close");
	std::string map1_creat_time_new;
	std::string map1_creat_time_old;
	std::string map2_creat_time_new;
	std::string map2_creat_time_old;
	http::client::response response = check_map.get(request);
	if (!map_list_reader.parse(body(response), map_list))
	{
		ROS_ERROR("get all map json parse failed\n");

	}
	else {
		if (!(map_list["msg"].asString() == "successed"))
		{
			ROS_ERROR("get maplist  HttpGet ask failed!\n");

		}
		else {
			std::cout << "map_list" << map_list["data"] << std::endl;
			for (int i = 0; i < map_list["data"].size(); i++)
			{
				if (map_list["data"][i]["name"].asString() == "map1")
					map1_creat_time_new = map_list["data"][i]["createdAt"].asString();
				if (map_list["data"][i]["name"].asString() == "map2")
					map2_creat_time_new = map_list["data"][i]["createdAt"].asString();
			}
			std::cout << "map1 new string is " << map1_creat_time_new <<"\n"<<std::endl;
			std::cout << "map2 new string is " << map2_creat_time_new <<"\n"<< std::endl;
			//check map1
			std::ifstream fin("/home/nvidia/map1_old.txt");
			std::ostringstream sin;
			sin << fin.rdbuf();
			std::string map1_string = sin.str();
			std::cout << "map1 old string is " << map1_string << "\n" << std::endl;
			if (!fin) {
				printf("The map creat_time file is not exist! \n");
				std::ofstream fout("/home/nvidia/map1_old.txt");
				fout << map1_creat_time_new;
				plist_param.map_check_index = true;
				fout.close();
				fin.close();
			}
			else 
			{
				printf("The map creat_time file is exist \n");
				std::ofstream fout("/home/nvidia/map1_old.txt");
				if (map1_string == map1_creat_time_new)
					plist_param.map_check_index = false;
				else if(map1_string != map1_creat_time_new )
					plist_param.map_check_index = true;
				fout << map1_creat_time_new;
				fout.close();
				fin.close();
			}

			//check map2 
			std::ifstream fin2("/home/nvidia/map2_old.txt");
			std::ostringstream sin2;
			sin2 << fin2.rdbuf();
			std::string map2_string = sin2.str();
			std::cout << "map2 old string is " << map2_string << "\n" << std::endl;
			if (!fin2) {
				printf("The map creat_time file is not exist! \n");
				std::ofstream fout2("/home/nvidia/map2_old.txt");
				fout2 << map2_creat_time_new;
				plist_param.map_check_index = true;
				fout2.close();
				fin2.close();
			}
			else
			{
				printf("The map creat_time file is exist \n");
				std::ofstream fout2("/home/nvidia/map2_old.txt");
				if (map2_string == map2_creat_time_new)
					plist_param.map_check_index = false;
				else if (map2_string != map2_creat_time_new)
					plist_param.map_check_index = true;
				fout2 << map2_creat_time_new;
				fout2.close();
				fin2.close();
			}

			}
		}
}
/*==============fuction for get the laser data and judge the map update status==============*/
void get_laser_data(const sensor_msgs::LaserScan &laser_scan)
{
	//ROS_INFO("TO GET THE laser information");
	laser_info.laser_detect = false;
	//printf("%s \n",laser_root);
	laser_info.frame_id = laser_scan.header.frame_id;
	laser_info.angle_min = laser_scan.angle_min;
	laser_info.angle_max = laser_scan.angle_max;
	laser_info.range_min = laser_scan.range_min;
	laser_info.range_max = laser_scan.range_max;
	//printf("%d \n", laser_scan.ranges.size());
	//laser_tmp.ranges[laser_root["ranges"].size()];
	for (int a = 0; a < laser_info.ranges.size(); a++)
	{
		laser_info.ranges[a] = laser_scan.ranges[a];
		if (a >= (15 + three_m_deta * 2) && a < (390 - 15 - three_m_deta * 2))
		{
			if (laser_info.ranges[a] > 3 && laser_info.ranges[a] < 5 && sta.status == 1 )
				laser_info.laser_detect = true;
		}

	}
	/* ===========print the laser ranges========== */
	//printf("%f \n",laser_root["ranges"][1].asFloat());
	//laser_tmp.ranges = laser_root["ranges"].asString();
	//printf("%s \n",laser_tmp);

}
void get_robot_speed(const chassis_control::chassis_sensor_data::ConstPtr& chassis_data)
{ robot_speed = chassis_data->rear_wheel_speed;
}
int main(int argc, char **argv)
{	modevar = getenv("RUN_MODE");
  	if(modevar!=NULL)
  	{
    std::cout<<"get envar $RUN_MODE success in test_node_main.cpp"<<std::endl;
    debugMode = strcmp(modevar,"debug")==0?true:false;
  	}
  	else
  	{
    std::cout<<"!!!get envar $RUN_MODE failed in test_node_main.cpp"<<std::endl;
    debugMode = false;
  	}
  	if(debugMode)
    std::cout<<"Run in debug mode!"<<std::endl;
  	else
    std::cout<<"Run in release mode!"<<std::endl;
	
	ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
//	Ping ping = Ping();
//	PingResult pingResult;
	ros::ServiceServer service1, service2, service3, service4, service5, service6, service7,service8,service9,service10,service11,service12,service13;
//initialize navi module ip, port, websocket port, websocket url, homereturn map name, homereturn path name
	gStrIp                = argv[1];
    gStrStatePort         = argv[2]; 
    gStrUrl               = argv[3];
    gStrPort              = argv[4]; 
	strHomeMapName        = argv[5]; 
	strHomePathName       = argv[6];
	RECORD_PATH           = atoi(argv[7]);
	strIp                 = "http://" + gStrIp +":" + gStrPort;
//gaussian api interface string
	strMapListGet 		  = strIp + "/gs-robot/data/maps";
	strMapDataGet 		  = strIp + "/gs-robot/data/map_png?map_name=";
	strPathGet 		      = strIp + "/gs-robot/data/paths?map_name=";
	strRealtimePosGet 	  = strIp + "/gs-robot/real_time_data/position";
	strCurrentMapGet 	  = strIp + "/gs-robot/real_time_data/current_initialize_status";
	strSuspendNavigation  = strIp + "/gs-robot/cmd/pause_navigate";
	strCancelNavigation   = strIp + "/gs-robot/cmd/cancel_navigate";
	strResumeNavigation   = strIp + "/gs-robot/cmd/resume_navigate";
	strTaskExecute		  = strIp + "/gs-robot/cmd/start_task_queue";
    strPathContentGet1    = strIp + "/gs-robot/data/path_data_list?map_name=";
	strPathContentGet2    =  "&path_name=";
	strWorkStatus         = strIp + "/gs-robot/real_time_data/work_status";
	strLoadMap            = strIp + "/gs-robot/cmd/load_map?map_name=";
    strStaticInitial1     = strIp + "/gs-robot/cmd/initialize_directly?map_name=";
	strStaticInitial2     = "&init_point_name=Origin";
	strGpsPos             = strIp + "/gs-robot/real_time_data/gps_raw";
	std::string strRobot_param_list     = strIp + "/gs-robot/data/robot_param_list";
	std::string strModify_robot_param   = strIp + "/gs-robot/cmd/modify_robot_param";
	laserstatus          = strIp + "/gs-robot/real_time_data/laser_raw";
	strCancelstask = strIp + "/gs-robot/cmd/stop_task_queue";
	strSuspendstask = strIp + "/gs-robot/cmd/pause_task_queue";
	strResumestask = strIp + "/gs-robot/cmd/resume_task_queue";
	strCurrentinit = strIp      +"/gs-robot/cmd/initialize_customized_directly";
	strSetspeedlevel = strIp +"/gs-robot/cmd/set_speed_level?level=";
	strPlanningbstacle = strIp +"/gs-robot/cmd/update_virtual_obstacles?map_name=";
	n.getParam("loop_count",loop_count_tmp);
	n.getParam("destination_goal",destination_goal_tmp);
	n.getParam("planning_obstacles",planning_obstacles_tmp);
	//n.getParam("origin_postion",origin_postion_tmp);
	//std::cout <<" origin postion is "<<std::string(origin_postion_tmp["map_name"])<< std::endl;
	std::cout <<" the navigation goal is "<<std::string(destination_goal_tmp[0]["position_name"][0])<<std::string(destination_goal_tmp[1]["position_name"][0])<< std::endl;
	three_m_deta = atan(300 / (laser_distance_x + 35)) * 180 / 3.1416;
//initialize lock
	pthread_mutex_init(&g_mutex, NULL);  

	Http = new HttpRequest;

//thread for monitoring gaussian task state
 	pthread_t tid;
    void *ret;
	sta.status = sta.IDLE;
    int err    = pthread_create(&tid, NULL, thread_GetRobotState, NULL);
    if (err != 0)
    {
        ROS_ERROR("pthread_create failed\n");
        return err;
    }
	
    
//initialize service and topic 
	test::movepos pos;
	test::gps_tx2 gps_tx2_param;
	test::plist   plist;
	pos.x = pos.y= pos.theta= 0;
	ros::Publisher map_robotpos;
	ros::Publisher map_robotstatus;
	ros::Publisher map_maplist;
	ros::Subscriber suspend_cmd;
	ros::Subscriber laser_sub;
	ros::Subscriber resume_cmd;
	ros::Publisher laser_status;
	ros::Publisher tx2_gps;
	ros::Publisher map_list_pub;
	ros::Subscriber gps_rcu;
	ros::Subscriber gps_raw;
	ros::Subscriber chassis_sensor_data_sub;
	laser_distance_x = 20;
	map_robotpos    = n.advertise<test::movepos>("movement_pose", 100, true);
	map_robotstatus = n.advertise<test::status>("movement_state", 100, true);
	laser_status = n.advertise<test::laser>("laser_status",100,true);
	tx2_gps = n.advertise<test::gps_tx2>("gps_tx2",100,true);
	map_list_pub = n.advertise<test::plist>("map_check",100,true);
	suspend_cmd = n.subscribe("suspend",100,suspend_callback);
	resume_cmd = n.subscribe("resume",100,resume_callback);
	chassis_sensor_data_sub =n.subscribe("chassis_sensor_data",100,get_robot_speed);
	gps_raw = n.subscribe("gps",100,GTK_gps_callback);
	gps_rcu = n.subscribe("gps_rcu",100,gps_fusion);
	laser_sub = n.subscribe("scan", 100, get_laser_data);
	 
	
	

	service1        = n.advertiseService("static_map1", mapCallback1); //map1 service
	service2        = n.advertiseService("static_map2", mapCallback2); // map2 service

	service3        = n.advertiseService("map_list", mapCallback3); // map list service
	service4        = n.advertiseService("path_list", mapCallback4); // get the pathlist service

	service5        = n.advertiseService("gps_pos", mapCallback5);  // get the gps service
	
    service6        = n.advertiseService("cancelPatrol", mapCallback6); // cancel navigation service

	service7        = n.advertiseService("suspendPatrol", mapCallback7); // suspend navigation service
	service8        = n.advertiseService("resumePatrol", mapCallback8);  // resume navigation service

	service9        = n.advertiseService("PatrolTaskExecute", mapCallback9); // start navigation service
	service10       = n.advertiseService("HomeReturn", mapCallback10); // return home or charging station service
	service11       = n.advertiseService("map_1_base64", mapCallback11); //encode map1 base64 service
	service12       = n.advertiseService("map_2_base64", mapCallback12); //encode map2 base64 service
	//service13       = n.advertiseService("init_the_map",init_the_map_callback);// init the robot origin position and the gps information
//	test::stdSrv::Request req;test::stdSrv::Response res; 
//	mapCallback3(req,res );
//  map1();
	//ros::Subscriber rossub_gps = n.subscribe("gps", 10, dataCallBack);
	   
	
//	test::plist   plist;
//  maplist(plist);
	check_map_diff(plist);
	map_list_pub.publish(plist);
/*===================init the speed setting and the oavoid obstacle mode for robot ====================*/
	ros::Rate loop_rate(4);
	Json::Value  avoid_obstacle;
	Json::Value  modify_param;
	Json::Value  avoid_temp;
	Json::Reader avoid_obstacle_reader;
	Json::FastWriter avoid_obstacle_writer;
	Json::Reader modify_reader;
	http::client client_modify;	  
	http::client::request request(strRobot_param_list);
	request << header("Connection", "close");
	http::client::response response = client_modify.get(request);
	if(!avoid_obstacle_reader.parse(body(response),avoid_obstacle))
	{
		ROS_ERROR("get robot status  json parse failed\n");
	
	}
	else{
	if(!(avoid_obstacle["msg"].asString() == "successed"))
	{
		ROS_ERROR("get robotstatus  HttpGet ask failed!\n");	
		
	}
	else{
		Json::Value data = avoid_obstacle["data"];
		std::cout << "old_param"<< avoid_obstacle["data"] << std::endl;
		for(int i=0;i<data.size();i++){
		if ((avoid_obstacle["data"][i]["namespace"].asString() == "/strategy/follow_base/avoid_obstacle"))
		
		{avoid_obstacle["data"][i]["value"]="true";
		avoid_temp=avoid_obstacle["data"][i];
		std::cout << "new_param for follow_base"<< avoid_temp<< std::endl;
		}
		if ((avoid_obstacle["data"][i]["namespace"].asString() == "/strategy/track_base/avoid_obstacle"))

		{
			avoid_obstacle["data"][i]["value"] = "true";
			avoid_temp = avoid_obstacle["data"][i];
			std::cout << "new_param for track_base" << avoid_temp << std::endl;
		}
			}
	}
	}
	std::string avoid_obstacle_temp = avoid_obstacle_writer.write(avoid_obstacle["data"]);
	//http::client client_modify
	http::client::request request_modify(strModify_robot_param);
	request_modify<<header("Connection", "close");
	response= client_modify.post(request_modify,avoid_obstacle_temp);
	if(!avoid_obstacle_reader.parse(body(response),modify_param))
	{
		ROS_ERROR("modify robot status  json parse failed\n");
	
	}	
	std::cout << "modify_param"<< body(response) << std::endl;
	if(!(modify_param["msg"].asString() == "successed"))
	{
		ROS_ERROR("get paramstatus  HttpGet ask failed!\n");	
		
	}
	//request.close();
	//request_modify.close();
	//response.close();
	//client_modify.close();
	std::string robot_speed = strSetspeedlevel +"2";
	http::client::request set_robot_speed(robot_speed);
	set_robot_speed << header("Connection", "close");
	http::client::response set_robot_response = client_modify.get(set_robot_speed);
	if(!avoid_obstacle_reader.parse(body(set_robot_response),avoid_obstacle))
	{
		ROS_ERROR("get robot speed json parse failed\n");
	
	}
	else{
	if(!(avoid_obstacle["msg"].asString() == "successed"))
	{
		ROS_ERROR("set robot speed  HttpGet ask failed!\n");	
		
	}}
/*============================INIT THE ROBOT ORIGIN POSITION====================================*/
	std::string origin_postion_str;
	std::string origin_path;
	int x0,y0,x2,y2;
	n.getParam("origin_postion/map_name",origin_postion_str);
	n.getParam("origin_postion/origin_latitude",origin_latitude);
	n.getParam("origin_postion/origin_longtitude",origin_longtitude);
	n.getParam("origin_postion/origin_bearing",origin_bearing);
	n.getParam("origin_postion/origin_path",origin_path);
	n.getParam("origin_postion/switch",origin_switch);
	//std::string origin_postion_str = std::string(origin_postion_tmp["map_name"]);
	//double origin_latitude_tmp = double(origin_postion_tmp["origin_latitude"]);
	//double origin_longtitude_tmp = double(origin_postion_tmp["origin_longtitude"]);
	//double origin_bearing_tmp = double(origin_postion_tmp["origin_bearing"]);
	//origin_latitude = origin_latitude_tmp;
	//origin_longtitude = origin_longtitude_tmp;
	//origin_bearing = origin_bearing_tmp;
	std::string str_origin =strIp +"/gs-robot/data/positions?map_name="+ origin_postion_str +"&type=0";
	//std::string str_origin =strIp +"/gs-robot/data/positions?map_name="+ "map2" +"&type=0";
	//std::cout << "the str of origin is"<< str_origin<< std::endl;
	http::client::request request_origin(str_origin);
  	request_origin<<header("Connection", "close");
  	response= client_modify.get(request_origin);
	if(!reader.parse(body(response),root))
	{
		ROS_ERROR("origin robot pose json parse failed\n");
		return false;
	}
	for(int index=0;index<root["data"].size();index++)
		{
		if (root["data"][index]["name"].asString()=="Origin")
		{origin_x = root["data"][index]["gridY"].asFloat();
		origin_y = root["data"][index]["gridX"].asFloat();
		origin_angle = root["data"][index]["angle"].asFloat();}
		}
	std::string path_data_tmp ="/home/nvidia/"+origin_postion_str+"_path_list.txt";
	std::ifstream path_list_param_ifstr (path_data_tmp);
	//std::cout << "the str of origin is "<< path_list_param_ifstr << std::endl;
	if(!reader.parse(path_list_param_ifstr,root))
	{
		ROS_ERROR(" path list info json parse failed\n");
		//return false;
	}

	if(!root["successed"].asBool())
	{
		ROS_ERROR(" obastacle path list info  get failed!\n");	
		//return false;
	}
	ROS_INFO("the root size is %d",root["data"].size());
	for(int i=0; i<root["data"].size();i++)
		{if(root["data"][i]["name"]==origin_path)
		{
		x0=root["data"][i]["gridPosition"][1]["y"].asInt();
		y0=root["data"][i]["gridPosition"][1]["x"].asInt();
		x2=root["data"][i]["gridPosition"][3]["y"].asInt();
		y2=root["data"][i]["gridPosition"][3]["x"].asInt();
		}
		ROS_INFO("the x y are %d %d %d %d",x0,y0,x2,y2);
		if(x2>=x0)
			origin_shifting_angle = atan((y2-y0)/(x2-x0))*180/3.1416;
		if(x2<x0)
			origin_shifting_angle = 180+atan((y2-y0)/(x2-x0))*180/3.1416;
		}
/*==========================END THE ROBOT ORIGIN POSTION==================================*/
/*============enter the loop mode ============= */
	while(ros::ok())	
	{

		bRet = robotpos(pos);
		if(!bRet)
			ROS_ERROR("robotpos get failed\n");

		//bRet = maplist(plist);
		//if(!bRet)
		//	ROS_ERROR("maplist get failed\n");
		//printf("%d,%d,%d",pos.x,pos.y,pos.theta);
		if(debugMode)
		ROS_INFO("the real_latitude is %f \n",real_latitude);
		if (sta.status == 1 || sta.status == 4 || sta.status == 5 || sta.status == 6)
		{
			gps_tx2_param.navigation_model = 1; // gaussion navitation model
			navigation_model_status = 1;
		}
		if(sta.status ==7 && gps_status ==2 && navigation_model_status==1)
			{gps_tx2_param.navigation_model=2;// gps navigation model
		}
		if (sta.status == 7 && gps_status != 2 && navigation_model_status == 1)
		{
			gps_tx2_param.navigation_model = 3;// lost localization and gps is not good; no action
		}
		gps_tx2_param.tx2_longtitude =real_longtitude;
		gps_tx2_param.tx2_latitude = real_latitude; 
		gps_tx2_param.tx2_altitude = real_altitude;
		gps_tx2_param.gps_longtitude = flongitude;
		gps_tx2_param.gps_latitude = flatitude;
		gps_tx2_param.gps_altitude = faltitude;
		gps_tx2_param.gps_status = gps_status; // status==2 is ok
		gps_tx2_param.transformLon = transformLon_current;
		gps_tx2_param.transformLat = transformLat_current;
		//get_laser_data(laser_info);
		tx2_gps.publish(gps_tx2_param);
		map_robotpos.publish(pos);
		laser_status.publish(laser_info);
		pthread_mutex_lock(&g_mutex);
		map_robotstatus.publish(sta);
		pthread_mutex_unlock(&g_mutex);
		//map_maplist.publish(plist);
		
		//plist.count = 0;
		//plist.list.dim.clear();
		loop_rate.sleep();
		ros::spinOnce();
		
	}/*
*/
    //ros::spin();
	ROS_INFO("quit\n");
	return 0;
}
