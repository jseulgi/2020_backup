#include "map_generation/map_generation.h"


MapGen::MapGen()
{
    SetParams();
    if(is_lane_on)
        LaneArrayPub = nh.advertise<visualization_msgs::MarkerArray>("lane_array",1,true);
    if(is_node_on)
        MapPointsPub = nh.advertise<visualization_msgs::Marker>("nodes",1, true);
    if(is_link_on)
        LinkArrayPub = nh.advertise<visualization_msgs::MarkerArray>("link_array",1, true);
    if(is_path_on){
        OffsetMapPub = nh.advertise<geometry_msgs::Point>("/OffsetMap",1, true);
    }

    // ros::Duration(2.0).sleep();
    ros::Rate rate(100);

    OpenLaneSHP();
    OpenLinkSHP();
    OpenNodeSHP();

    if (is_lane_on && is_node_on && is_link_on && is_path_on){
      GlobalMapPub = nh.advertise<Map>("/map_info", 1, true);
      ROS_INFO_STREAM("Map information published");
      GlobalMapPub.publish(GlobalMap);
    }
}

MapGen::~MapGen()
{

}



void MapGen::SetParams()
{
    nh = ros::NodeHandle("~");
    is_simul_on = false;
    nh.param("/rviz_on/is_link_on", is_link_on, true);
    nh.param("/rviz_on/is_node_on", is_node_on, true);
    nh.param("/rviz_on/is_lane_on", is_lane_on, true);
    nh.param("/rviz_on/is_path_on", is_path_on, true);

    nh.param<std::string>("/abspath",abspath,"what");

    nh.param<std::string>("/shp/lane", lane_shp, "/home/jsg/AutoCar/Material/[0] HdMap/HD_Map/KIAPI Vector HD Map/A1_LANE/A1_LANE.shp");
    nh.param<std::string>("/shp/link", link_shp, "/home/jsg/AutoCar/Material/[0] HdMap/HD_Map/KIAPI Vector HD Map/A3_LINK/A3_LINK.shp");
    nh.param<std::string>("/shp/node", node_shp, "/home/jsg/AutoCar/Material/[0] HdMap/HD_Map/KIAPI Vector HD Map/C1_NODE/C1_NODE.shp");

    lane_shp = abspath + lane_shp;
    link_shp = abspath + link_shp;
    node_shp = abspath + node_shp;

    nh.param<std::string>("/layer/lane", layer_lane, "A1_LANE");
    nh.param<std::string>("/layer/link", layer_link, "A3_LINK");
    nh.param<std::string>("/layer/node", layer_node, "C1_NODE");

    nh.param("/lane/id", shp_lane_id, 9);
    nh.param("/lane/right_link", shp_lane_right_link, 0);
    nh.param("/lane/left_link", shp_lane_left_link, 1);
    nh.param("/lane/lanetype", shp_lane_lanetype, 2);
    nh.param("/lane/lanecode", shp_lane_lanecode, 3);
    nh.param("/lane/barrier", shp_lane_barrier, 4);
    nh.param("/lane/laneno", shp_lane_laneno, 5);
    nh.param("/lane/code", shp_lane_code, 6);

    nh.param("/link/id", shp_link_id, 0);
    nh.param("/link/from_node", shp_link_from_node, 1);
    nh.param("/link/to_node", shp_link_to_node, 2);
    nh.param("/link/length", shp_link_length, 3);
    nh.param("/link/road_type", shp_link_road_type, 4);
    nh.param("/link/speed", shp_link_speed, 6);
    nh.param("/link/left_link", shp_link_left_link, 16);
    nh.param("/link/right_link", shp_link_right_link, 17);
    nh.param("/link/next_link", shp_link_next_link, 18);

    nh.param("/node/id", shp_node_id, 0);
    nh.param("/node/next_link", shp_node_next_link, 8);
    nh.param("/node/prev_link", shp_node_prev_link, 9);

    ROS_INFO("LANE_SHAPEFILE : \n%s\n", lane_shp.c_str());
    ROS_INFO("LINK_SHAPEFILE : \n%s\n", link_shp.c_str());
    ROS_INFO("NODE_SHAPEFILE : \n%s\n", node_shp.c_str());
    ROS_INFO("Layer:\n%s  %s  %s  \n", layer_lane.c_str(), layer_link.c_str(), layer_node.c_str());
}

/** Open shape file of lane */
void MapGen::OpenLaneSHP(){

  	OGRRegisterAll();
    GDALDataset *poDS = static_cast<GDALDataset*>( GDALOpenEx( lane_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL ));

    if( poDS == NULL ){
        printf( "Open lane failed.\n" );
        exit( 1 );}

    OGRLayer  *poLayer = poDS->GetLayerByName( layer_lane.c_str() );
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;
    while( (poFeature = poLayer->GetNextFeature()) != NULL ){
      Lane lane;
      // ROS_INFO_STREAM("-----");
      for( int iField = 0; iField < poFDefn->GetFieldCount(); iField++ ){

        // cout <<"percent done:"<<iField<<"/"<<poFDefn->GetFieldCount() << endl;

        OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn( iField );
        string str = "";
        str = ParsingToString( poFieldDefn, poFeature, iField);
        unsigned long comma;
        string str_temp;
        // cout << str << endl;
        if ( iField == shp_lane_id ){
          lane.id = str;
        } else if ( iField == shp_lane_right_link ){
          lane.R_LINKID = str;
        } else if ( iField == shp_lane_left_link ){
          lane.L_LINKID = str;
        } else if ( iField == shp_lane_lanetype ){
          lane.lane_type = atoi( str.c_str() );
        } else if ( iField == shp_lane_lanecode ){
          lane.lane_code = atoi( str.c_str() );
        } else if ( iField == shp_lane_barrier ){
          lane.barrier = atoi( str.c_str() );
        } else if ( iField == shp_lane_laneno ){
          lane.lane_no = atoi( str.c_str() );
        } else if ( iField == shp_lane_code ){
          lane.code = atoi( str.c_str() );
        } else {
          // cout << "an unexpected result" << endl;
        }
      }

      // ROS_INFO_STREAM("checking the results ...");
      // cout << lane.id << endl;
      // cout << lane.R_LINKID << endl;
      // cout << lane.L_LINKID << endl;
      // cout << lane.lane_type << endl;
      // cout << lane.lane_code << endl;
      // cout << lane.barrier << endl;
      // cout << lane.lane_no << endl;
      // cout << lane.code << endl;
      // cout << "....." << endl;

      char *pszWKT = NULL;
      string geometry_str;
      vector<geometry_msgs::Point> lane_point_array;
      OGRGeometry *poGeometry = poFeature->GetGeometryRef();
      if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString ){
         	poGeometry->exportToWkt(&pszWKT);
          geometry_str = pszWKT;
         	// printf("%s\n", pszWKT);
         	CPLFree(pszWKT);
      }
      else{
          printf( "LANE : no point geometry\n" );}

      // std::cout << geometry_str << std::endl;
      lane_point_array = ParsingLaneGeometry(geometry_str);
      for(int i = 0; i < lane_point_array.size(); i++)
      {
        lane.geometry.push_back(lane_point_array[i]);
      }

      lanes.insert(make_pair(lane.id, lane));
      OGRFeature::DestroyFeature( poFeature );

    }
    GDALClose( poDS );

    InitializeRvizLaneArray();

    if(is_lane_on){
        // add lanes to global map
        for(auto it = lanes.begin() ; it != lanes.end(); ++it)
        {
          GlobalMap.lanes.push_back(it->second);
        }
        LaneArrayPub.publish(LaneArray);
    }

    ROS_INFO("Received Lane : %d", LaneArray.markers.size());
}

void MapGen::InitializeRvizLaneArray(){
  int i = 0;

  LaneArray.markers.resize(lanes.size());

  for(auto it=lanes.begin(); it!=lanes.end();it++){
      LaneArray.markers[i].header.frame_id = "map";
      LaneArray.markers[i].header.stamp = ros::Time::now();
      LaneArray.markers[i].action = visualization_msgs::Marker::ADD;
      LaneArray.markers[i].pose.orientation.w = 1.0;
      LaneArray.markers[i].id = i;
      LaneArray.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
      LaneArray.markers[i].scale.x = 0.3;
      LaneArray.markers[i].color.r = 1.0;
      LaneArray.markers[i].color.g = 1.0;
      LaneArray.markers[i].color.b = 1.0;
      LaneArray.markers[i].color.a = 1.0;

      LaneArray.markers[i].points.resize(it->second.geometry.size());

      for(int j = 0; j< it->second.geometry.size(); j++){
        geometry_msgs::Point point;
        point.x = it->second.geometry[j].x - OffsetMapX;
        point.y = it->second.geometry[j].y - OffsetMapY;
        LaneArray.markers[i].points[j] = point;
      }
      i++;
  }
}
vector<geometry_msgs::Point> MapGen::ParsingLaneGeometry(string geometry_str)
{
    string str;
    unsigned long comma;
    unsigned long space;

    geometry_msgs::Point lane_point;
    vector<geometry_msgs::Point> lane_point_array;

    comma = geometry_str.find("(");
    geometry_str = geometry_str.substr(comma+1);
    comma = geometry_str.find(" ");
    str = geometry_str.substr(0,comma);
    OffsetMapX = atof(str.c_str());
    geometry_str = geometry_str.substr(comma+1);
    comma = geometry_str.find(" ");
    str = geometry_str.substr(0,comma);
    OffsetMapY = atof(str.c_str());
    comma = geometry_str.find(",");
    geometry_str = geometry_str.substr(comma+1);
    GlobalMap.OffsetMapX = OffsetMapX;
    GlobalMap.OffsetMapY = OffsetMapY;

    lane_point.x = OffsetMapX;
    lane_point.y = OffsetMapY;
    lane_point.z = 0.0;
    lane_point_array.push_back(lane_point);

    while(geometry_str.find(",") != string::npos){
        comma = geometry_str.find(",");
        str = geometry_str.substr(0,comma);
        space = str.find(" ");
        lane_point.x = atof(str.substr(0,space).c_str());
        str = str.substr(space+1);
        space = str.find(" ");
        lane_point.y = atof(str.substr(0,space).c_str());
        lane_point.z = 0;
        lane_point_array.push_back(lane_point);
        geometry_str = geometry_str.substr(comma+1);
    }
    space = geometry_str.find(" ");
    lane_point.x = atof(geometry_str.substr(0,space).c_str()) ;
    geometry_str = geometry_str.substr(space+1);
    space = geometry_str.find(" ");
    lane_point.y = atof(geometry_str.substr(0,space).c_str());
    lane_point.z = 0;
    lane_point_array.push_back(lane_point);

    return lane_point_array;
}

string MapGen::ParsingToString (OGRFieldDefn *poFieldDefn, OGRFeature *poFeature, int iField){

    string str = "";
    switch( poFieldDefn->GetType() )
    {
        case OFTInteger:
            // printf( "%d,", poFeature->GetFieldAsInteger( iField ) );
            str.append( to_string( poFeature->GetFieldAsInteger(iField) ) );
            break;
        case OFTInteger64:
            // printf( CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64( iField ) );
            str.append( to_string( poFeature->GetFieldAsInteger64(iField) ) );
            break;
        case OFTReal:
            // printf( "%.3f,", poFeature->GetFieldAsDouble(iField) );
            str.append( to_string( poFeature->GetFieldAsDouble(iField) ) );
            break;
        case OFTString:
            // printf( "%s,", poFeature->GetFieldAsString(iField) );
            str.append( poFeature->GetFieldAsString(iField ) );
            break;
        default:
            // printf( "%s,", poFeature->GetFieldAsString(iField) );
            str.append( poFeature->GetFieldAsString(iField) );
            break;
    }
    return str;
}

/** Open shape file of node */
void MapGen::OpenNodeSHP(){

	OGRRegisterAll();
    GDALDataset *poDS = static_cast<GDALDataset*>( GDALOpenEx( node_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL ));
    if( poDS == NULL ){
        printf( "Open node failed.\n" );
        exit( 1 );}

    OGRLayer  *poLayer = poDS->GetLayerByName( layer_node.c_str() );  //"C1_NODE"
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;
    while( (poFeature = poLayer->GetNextFeature()) != NULL ) {
    	Node node;
        for( int iField = 0; iField < poFDefn->GetFieldCount(); iField++ )
        {
            OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn( iField );
            string str = "";
            str = ParsingToString( poFieldDefn, poFeature, iField);
            unsigned long comma;
            string str_temp;
            if ( iField == shp_node_id ){
                node.id = str;
                // cout << "Node ID : " << node.id << endl;
            }
            else if ( iField == shp_node_next_link ){
                str_temp = str;
                while(str.find(",") != string::npos){
                    comma = str.find(",");
                    str_temp = str.substr(0, comma);
                    node.NLIDS.push_back(str_temp);
                    str = str.substr(comma+1);
                }
                node.NLIDS.push_back(str);
                // cout << "NEXT LID : " << str << endl;
            }
            else if ( iField == shp_node_prev_link){
                str_temp = str;
                while(str.find(",") != string::npos){
                    comma = str.find(",");
                    str_temp = str.substr(0, comma);
                    node.PLIDS.push_back(str_temp);
                    str = str.substr(comma+1);
                }
                node.PLIDS.push_back(str);
                // cout << "PREV LID : " << str << endl;
            }
        }

        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
        if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbPoint ){
            OGRPoint *poPoint = (OGRPoint *) poGeometry;
            node.geometry.x = poPoint->getX();
            node.geometry.y = poPoint->getY();

            geometry_msgs::Point p;
            p.x = node.geometry.x - OffsetMapX;
            p.y = node.geometry.y - OffsetMapY;
            MapPoints.points.push_back(p);
            // printf( "%.3f,%3.f\n", poPoint->getX(), poPoint->getY() );
        }
        else{
            printf( "NODE: no point geometry\n" );}

      OGRFeature::DestroyFeature( poFeature );
		  nodes.insert(make_pair(node.id, node));
    }
    GDALClose( poDS );

    InitializeRvizPoints();

    if(is_node_on){

      // add nodes to global map
      for (auto it = nodes.begin(); it != nodes.end(); ++it){
        GlobalMap.nodes.push_back(it->second);
      }
      MapPointsPub.publish(MapPoints);
    }

   	ROS_INFO("Received Nodes : %d", nodes.size());
}

void MapGen::InitializeRvizPoints(){
    MapPoints.header.frame_id = "map";
    MapPoints.header.stamp = ros::Time::now();
    MapPoints.action = visualization_msgs::Marker::ADD;
    MapPoints.pose.orientation.w = 1.0;
    MapPoints.id = 1000;;
    MapPoints.type = visualization_msgs::Marker::POINTS;
    MapPoints.scale.x = 1.5;
    MapPoints.scale.y = 1.0;
    MapPoints.color.r = 1.0;
    MapPoints.color.g = 1.0;
    MapPoints.color.b = 0.0;
    MapPoints.color.a = 1.0;
}

/** Open shape file of link */
void MapGen::OpenLinkSHP(){

	OGRRegisterAll();
    vector<geometry_msgs::Point> LinkGeoVector;
    GDALDataset *poDS = static_cast<GDALDataset*>( GDALOpenEx( link_shp.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL ));
    if( poDS == NULL ){
        printf( "Open link failed.\n" );
        exit( 1 ); }

    OGRLayer  *poLayer = poDS->GetLayerByName( layer_link.c_str() ); //A3_LINK //"A3_LINK_Modified_V2"
    OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
    poLayer->ResetReading();
    OGRFeature *poFeature;

    while( (poFeature = poLayer->GetNextFeature()) != NULL ){
    	Link link;
      for( int iField = 0; iField < poFDefn->GetFieldCount(); iField++ ){
          OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn( iField );
          string str = "";
          str = ParsingToString( poFieldDefn, poFeature, iField);
          unsigned long comma;
          string str_temp;
          if ( iField == shp_link_id){
              link.id = str;
              // cout << endl << "LINK ID : " << link.id << endl;
          }
          else if( iField == shp_link_from_node ){
              link.from_node = str;
              // cout << "FROM NODE : " << link.from_node << endl;
          }
          else if ( iField == shp_link_to_node ){
              link.to_node = str;
              // cout << "To NODE : " << link.to_node << endl;
          }
          else if ( iField == shp_link_length ){
              link.length = atof( str.c_str() );
              // cout << "LEN : " << link.length << endl;
          }
          else if (iField == shp_link_road_type){
            link.road_type = atoi(str.c_str());
          }
          else if ( iField == shp_link_speed ){
              link.speed = atoi( str.c_str() );
              // cout << "SPEED : " << link.speed << endl;
          }
          else if ( iField == shp_link_left_link ){
              link.LLID.push_back( str );
              // cout<< "L LINK : " << str << endl;
          }
          else if ( iField == shp_link_right_link ){
              link.RLID.push_back( str );
              // cout<< "R LINK : " << str << endl;
          }
          else if ( iField == shp_link_next_link ){
              str_temp = str;
              while(str.find(",") != string::npos){
              comma = str.find(",");
              str_temp = str.substr(0, comma);
              link.NLIDS.push_back(str_temp);
              str = str.substr(comma+1);
              }
              link.NLIDS.push_back(str);
              // for(int i =0; i<link.NLIDS.size(); i++)
              // cout<< "NEXT LINK : " << link.NLIDS[i] << endl;
          }
      }

      char *pszWKT = NULL;
      string geometry_str;
      vector<geometry_msgs::Point> link_array;
      OGRGeometry *poGeometry = poFeature->GetGeometryRef();
      if( poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString ) {
         	poGeometry->exportToWkt(&pszWKT);
          geometry_str = pszWKT;
         	//printf("%s\n", pszWKT);
         	CPLFree(pszWKT);
      }
      else {
          printf( "LINK : no point geometry\n" );}

      link_array = ParsingLinkGeometry(geometry_str);
      for(int i=0; i<link_array.size(); i++){
          link.geometry.push_back(link_array[i]);
      }
  		OGRFeature::DestroyFeature( poFeature );

      // cout << link.id;

      // Link - lane relationship
      for (auto it = lanes.begin() ; it != lanes.end(); ++it){
        string lane_id = it->first;
        string rlink_id = it->second.R_LINKID;
        string llink_id = it->second.L_LINKID;

        if (rlink_id != "")
        {
         if (link.id == rlink_id)
         {
           link.L_laneID.push_back(lane_id);
           // cout << " L: ";
           // cout << lane_id;
         }
        }

        if (llink_id != "")
        {
         if (link.id == llink_id)
         {
           // cout << " R: ";
           // cout << lane_id;
           link.R_laneID.push_back(lane_id);
         }
        }
      }
  		links.insert(make_pair(link.id, link));
    }

    GDALClose( poDS );
    InitializeRvizLinkArray();

    if(is_link_on){

      // add links to global map
      for(auto it = links.begin() ; it != links.end(); ++it){
        GlobalMap.links.push_back(it->second);
      }

      LinkArrayPub.publish(LinkArray);
    }
   	ROS_INFO("Received Links : %d", links.size());
}

vector<geometry_msgs::Point> MapGen::ParsingLinkGeometry(string geometry_str){

    string str;
    unsigned long comma;
    unsigned long space;

    geometry_msgs::Point point;
    vector<geometry_msgs::Point> point_array;

    comma = geometry_str.find("(");
    geometry_str = geometry_str.substr(comma+1);

    while(geometry_str.find(",") != string::npos){
        comma = geometry_str.find(",");
        str = geometry_str.substr(0,comma);
        space = str.find(" ");
        point.x = atof(str.substr(0,space).c_str());
        str = str.substr(space+1);
        space = str.find(" ");
        point.y = atof(str.substr(0,space).c_str());
        point_array.push_back(point);
        geometry_str = geometry_str.substr(comma+1);
    }
    space = geometry_str.find(" ");
    point.x = atof(geometry_str.substr(0,space).c_str());
    geometry_str = geometry_str.substr(space+1);
    space = geometry_str.find(" ");
    point.y = atof(geometry_str.substr(0,space).c_str());
    point_array.push_back(point);

    return point_array;
}

void MapGen::InitializeRvizLinkArray(){

    int i = 0;

    LinkArray.markers.resize(links.size());

    for (auto it=links.begin(); it!=links.end(); it++){
        LinkArray.markers[i].header.frame_id = "map";
        LinkArray.markers[i].header.stamp = ros::Time::now();
        LinkArray.markers[i].action = visualization_msgs::Marker::ADD;
        LinkArray.markers[i].pose.orientation.w = 1.0;
        LinkArray.markers[i].id = i;
        LinkArray.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        LinkArray.markers[i].scale.x = 0.3;
        LinkArray.markers[i].color.r = 0.0;
        LinkArray.markers[i].color.g = 1.0;
        LinkArray.markers[i].color.b = 1.0;
        LinkArray.markers[i].color.a = 1.0;

        LinkArray.markers[i].points.resize(it->second.geometry.size());

        for(int k= 0; k < it->second.geometry.size(); k++){
            geometry_msgs::Point point;
            point.x = it->second.geometry[k].x - OffsetMapX;
            point.y = it->second.geometry[k].y - OffsetMapY;
            LinkArray.markers[i].points[k] = point;
        }
        i++;
    }
}
