#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <string>
#include <cmath>
#include <limits>
// Grid Map
#include "grid_map_msgs/GridMap.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include "grid_map_core/iterators/GridMapIterator.hpp"
// ROS
#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
// ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <xmlrpcpp/XmlRpcValue.h>

// STL
#include <algorithm>
// kindr
#include <kindr/Core>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace sensor_msgs;
using namespace message_filters;
using namespace grid_map;
using namespace Eigen;
using namespace std;

class Node
{
    public:
        Node()
        {
            sub_slope.subscribe(nh_, "/grid_map_filter_demo/filtered_map", 30);

            sub_step.subscribe(nh_, "/elevation_mapping/elevation_map", 40);
            sub_imu = nh_.subscribe("/imu", 1, &Node::callback1, this); // queue de 1 para que siempre se coja el valor que corresponde a ese momento ya que llegan varios valores cada segundo
            pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/mapobs", 30);
            double pitch;
            // Sincronizar los mapas para la llamada del callback
            sync_.reset(new Sync(MySyncPolicy(30), sub_slope, sub_step));
            sync_->registerCallback(boost::bind(&Node::callback2, this, _1, _2));
        }

        void callback1(const sensor_msgs::Imu::ConstPtr& input_imu)
        {
            // datos IMU
            double roll,yaw,pitch;
            tf::Quaternion cuaternio;
            tf::quaternionMsgToTF(input_imu->orientation,cuaternio);
            tf::Matrix3x3(cuaternio).getRPY(roll, pitch, yaw);
        }

        void callback2(const grid_map_msgs::GridMap::ConstPtr& input, const grid_map_msgs::GridMap::ConstPtr& input2)
        {
            ROS_INFO("entrando callback");
            // Grid map de slope
            grid_map::GridMap gridMap;
            GridMapRosConverter::fromMessage( *input,gridMap);
            // Grid map de elevacion
            grid_map::GridMap gridStep;
            GridMapRosConverter::fromMessage( *input2,gridStep);
            // Objetos tipo position
            Position position = gridMap.getPosition() - 0.5 *gridMap.getLength().matrix();//posicion en el frame (centro mapa)-longitud, por lo tanto el 0,0 deja de estar en el frame sino en la esquina de arriba a la izquierda
            
            Position positionnow, positionUP, positionDOWN, positionRIGHT, positionLEFT;

            // Crear mapa de costes con los mismos parametros que el de slope
            nav_msgs::OccupancyGrid output;
            output.header.frame_id = gridMap.getFrameId();
            output.header.stamp.fromNSec(gridMap.getTimestamp());
            output.info.map_load_time = output.header.stamp;
            output.info.resolution = gridMap.getResolution();
            output.info.width = gridMap.getSize()(0);
            output.info.height = gridMap.getSize()(1);
            output.info.origin.position.x = position[0];
            output.info.origin.position.y = position[1];
            output.info.origin.position.z = 0.0;
            output.info.origin.orientation.x = 0.0;
            output.info.origin.orientation.y = 0.0;
            output.info.origin.orientation.z = 0.0;
            output.info.origin.orientation.w = 1.0;
            size_t nCells = gridMap.getSize()prod();
            output.data.resize(nCells);
            const float resolution =gridMap.getResolution();

            // Valores que pueden tomar las casillas
            const int obs=100;//obstaculos
            const int desc=-1.0;//mapa desconocido
            const int blanco=0.0;//zonas libres
            // Valor inicial la casilla final del occupancy grid
            float casilla = 0.0;
            // Valor critico de rampa
            const float critical_slope = 0.1396; //8º
            // Valor critico de escalon
            const float critical_step_down = 0.10; //cm
            const float critical_step_up = 0.03; //cm
            for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator){
                if(gridMap.getPosition(*iterator, positionnow)==true){
                    float value = (gridMap.at("slope", *iterator)); //valor de rampa en cada casilla
                    float step = (gridStep.at("elevation", *iterator)); // valor de elevacion de cada casilla
                    // Actualizacion posiciones de las vecinas
                    float valueUP_step, valueDOWN_step, valueRIGHT_step, valueLEFT_step = -10.0;// valor que no va a tomar nunca para inicializar
                    positionUP[0] = positionnow[0];
                    positionUP[1] = positionnow[1]+resolution;
                    if(gridStep.isInside(positionUP)==true)
                    valueUP_step = (gridStep.atPosition("elevation", positionUP));
                    positionDOWN[0] = positionnow[0];
                    positionDOWN[1] = positionnow[1]-resolution;
                    if(gridStep.isInside(positionDOWN)==true)
                    valueDOWN_step = (gridStep.atPosition("elevation",
                    positionDOWN));
                    positionRIGHT[0] = positionnow[0]+resolution;
                    positionRIGHT[1] = positionnow[1];
                    if(gridStep.isInside(positionRIGHT)==true)
                    valueRIGHT_step = (gridStep.atPosition("elevation",
                    positionRIGHT));
                    positionLEFT[0] = positionnow[0]-resolution;
                    positionLEFT[1] = positionnow[1];
                    if(gridStep.isInside(positionLEFT)==true)
                    valueLEFT_step = (gridStep.atPosition("elevation",
                    positionLEFT));
                    if (isnan(value) && (((positionnow[0]>position[0]+5.5) &&
                    (positionnow[0]<position[0]+6.5)) && ((positionnow[1]>position[1]+5.5) &&
                    (positionnow[1]<position[1]+6.5)))){ //da valor a las zonas de alrededor al principio, el radio ciego del robot es aprox 0.5m, mapa de longitud 12x12m, el centro en 6m
                    
                    casilla = blanco;

                    }else if (isnan(value)|| valueUP_step == -10.0 || valueDOWN_step ==
                    -10.0 || valueRIGHT_step == -10.0 || valueLEFT_step == -10.0){// si no tiene valor asignado
                    casilla = desc;
                    }else if(((value + pitch) < critical_slope)&& ((value +
                    pitch)>=1.57)){// angulos menores que el critico y mayores de 90º, ya que puede bajar cualquier inclinacion
                    casilla = blanco;
                    }else if ((((step + critical_step_up >= valueUP_step )&&( step -
                    critical_step_down <= valueUP_step))&&((step + critical_step_up >=
                    valueDOWN_step )&&( step - critical_step_down <= valueDOWN_step))&&((step +
                    critical_step_up >= valueRIGHT_step )&&( step - critical_step_down <=
                    valueRIGHT_step))&&((step + critical_step_up >= valueLEFT_step )&&( step -
                    critical_step_down <= valueLEFT_step)))&&(valueUP_step != -10.0 &&
                    valueDOWN_step != -10.0 && valueRIGHT_step != -10.0 && valueLEFT_step != -
                    10.0)){// escalon de subida o bajada dentro de los limites establecidos y existe valor de las vecinas
                    casilla = blanco;
                    }else if ((((step + critical_step_up < valueUP_step )&&( step -
                    critical_step_down > valueUP_step))&&((step + critical_step_up <
                    valueDOWN_step )&&( step - critical_step_down > valueDOWN_step))&&((step +
                    critical_step_up < valueRIGHT_step )&&( step - critical_step_down >
                    valueRIGHT_step))&&((step + critical_step_up < valueLEFT_step )&&( step -
                    critical_step_down > valueLEFT_step)))&&(valueUP_step != -10.0 &&
                    valueDOWN_step != -10.0 && valueRIGHT_step != -10.0 && valueLEFT_step != -
                    10.0)){// escalon de subida o bajada considerados como obstaculo
                    casilla = obs;
                    }else if(((value + pitch) >= critical_slope)&&((value+pitch) <
                    1.57)){
                    casilla = obs;
                    }
                    size_t index =
                    getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(),
                    false);
                    //El orden entre los dos mapas de las celdas no es lo mismo por eso cambia del index 2dim a un indice 1dim
                    output.data[nCells - index - 1] = casilla;
                }
            }// Publicar salida en el mapa de costes
            pub_.publish(output);
            ROS_INFO("saliendo callback");
    }

    private:
    ros::NodeHandle nh_;
    // suscripciones
    message_filters::Subscriber<grid_map_msgs::GridMap> sub_slope;
    message_filters::Subscriber<grid_map_msgs::GridMap> sub_step;
    ros::Subscriber sub_imu;
    // publicacion
    ros::Publisher pub_;

    // sincronizacion
    typedef
    message_filters::sync_policies::ApproximateTime<grid_map_msgs::GridMap,grid_map_msgs::GridMap> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;// Sincronizacion de los topics a los que se suscribe el nodo, ya que tienen valores de actualizacion parecidos

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_and_publish");
    Node synchronizer;
    ros::spin();
    return 0;
}