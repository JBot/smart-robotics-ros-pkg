#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_eurobot_obstacles");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

  unsigned int num_points = 110;

  int count = 0;
  ros::Rate r(7.0);
  while(n.ok()){
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "/world";

    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    //generate some fake data for our point cloud
    // borders
      cloud.points[0].x = 0.25;
      cloud.points[0].y = 0.0;
      cloud.points[0].z = 0.0;
      cloud.channels[0].values[0] = 199;

      cloud.points[1].x = 0.5;
      cloud.points[1].y = 0.0;
      cloud.points[1].z = 0.0;
      cloud.channels[0].values[1] = 199;

      cloud.points[2].x = 0.75;
      cloud.points[2].y = 0.0;
      cloud.points[2].z = 0.0;
      cloud.channels[0].values[2] = 199;

      cloud.points[3].x = 1.0;
      cloud.points[3].y = 0.0;
      cloud.points[3].z = 0.0;
      cloud.channels[0].values[3] = 199;

      cloud.points[4].x = 1.25;
      cloud.points[4].y = 0.0;
      cloud.points[4].z = 0.0;
      cloud.channels[0].values[4] = 199;

      cloud.points[5].x = 1.5;
      cloud.points[5].y = 0.0;
      cloud.points[5].z = 0.0;
      cloud.channels[0].values[5] = 199;


      cloud.points[6].x = 0.25;
      cloud.points[6].y = 2.0;
      cloud.points[6].z = 0.0;
      cloud.channels[0].values[6] = 199;

      cloud.points[7].x = 0.5;
      cloud.points[7].y = 2.0;
      cloud.points[7].z = 0.0;
      cloud.channels[0].values[7] = 199;

      cloud.points[8].x = 0.75;
      cloud.points[8].y = 2.0;
      cloud.points[8].z = 0.0;
      cloud.channels[0].values[8] = 199;

      cloud.points[9].x = 1.0;
      cloud.points[9].y = 2.0;
      cloud.points[9].z = 0.0;
      cloud.channels[0].values[9] = 199;

      cloud.points[10].x = 1.25;
      cloud.points[10].y = 2.0;
      cloud.points[10].z = 0.0;
      cloud.channels[0].values[10] = 199;

      cloud.points[11].x = 1.5;
      cloud.points[11].y = 2.0;
      cloud.points[11].z = 0.0;
      cloud.channels[0].values[1] = 199;


      cloud.points[12].x = -0.25;
      cloud.points[12].y = 0.0;
      cloud.points[12].z = 0.0;
      cloud.channels[0].values[12] = 199;

      cloud.points[13].x = -0.5;
      cloud.points[13].y = 0.0;
      cloud.points[13].z = 0.0;
      cloud.channels[0].values[13] = 199;

      cloud.points[14].x = -0.75;
      cloud.points[14].y = 0.0;
      cloud.points[14].z = 0.0;
      cloud.channels[0].values[14] = 199;

      cloud.points[15].x = -1.0;
      cloud.points[15].y = 0.0;
      cloud.points[15].z = 0.0;
      cloud.channels[0].values[15] = 199;

      cloud.points[16].x = -1.25;
      cloud.points[16].y = 0.0;
      cloud.points[16].z = 0.0;
      cloud.channels[0].values[16] = 199;

      cloud.points[17].x = -1.5;
      cloud.points[17].y = 0.0;
      cloud.points[17].z = 0.0;
      cloud.channels[0].values[17] = 199;


      cloud.points[18].x = -0.25;
      cloud.points[18].y = 2.0;
      cloud.points[18].z = 0.0;
      cloud.channels[0].values[18] = 199;

      cloud.points[19].x = -0.5;
      cloud.points[19].y = 2.0;
      cloud.points[19].z = 0.0;
      cloud.channels[0].values[19] = 199;

      cloud.points[20].x = -0.75;
      cloud.points[20].y = 2.0;
      cloud.points[20].z = 0.0;
      cloud.channels[0].values[20] = 199;

      cloud.points[21].x = -1.0;
      cloud.points[21].y = 2.0;
      cloud.points[21].z = 0.0;
      cloud.channels[0].values[21] = 199;

      cloud.points[22].x = -1.25;
      cloud.points[22].y = 2.0;
      cloud.points[22].z = 0.0;
      cloud.channels[0].values[22] = 199;

      cloud.points[23].x = -1.5;
      cloud.points[23].y = 2.0;
      cloud.points[23].z = 0.0;
      cloud.channels[0].values[23] = 199;


      cloud.points[24].x = -1.5;
      cloud.points[24].y = 1.75;
      cloud.points[24].z = 0.0;
      cloud.channels[0].values[24] = 199;

      cloud.points[25].x = -1.5;
      cloud.points[25].y = 1.5;
      cloud.points[25].z = 0.0;
      cloud.channels[0].values[25] = 199;

      cloud.points[26].x = -1.5;
      cloud.points[26].y = 1.25;
      cloud.points[26].z = 0.0;
      cloud.channels[0].values[26] = 199;

      cloud.points[27].x = -1.5;
      cloud.points[27].y = 1.0;
      cloud.points[27].z = 0.0;
      cloud.channels[0].values[27] = 199;

      cloud.points[28].x = -1.5;
      cloud.points[28].y = 0.75;
      cloud.points[28].z = 0.0;
      cloud.channels[0].values[28] = 199;

      cloud.points[29].x = -1.5;
      cloud.points[29].y = 0.5;
      cloud.points[29].z = 0.0;
      cloud.channels[0].values[29] = 199;

      cloud.points[30].x = -1.5;
      cloud.points[30].y = 0.25;
      cloud.points[30].z = 0.0;
      cloud.channels[0].values[30] = 199;


      cloud.points[31].x = 1.5;
      cloud.points[31].y = 1.75;
      cloud.points[31].z = 0.0;
      cloud.channels[0].values[31] = 199;

      cloud.points[32].x = 1.5;
      cloud.points[32].y = 1.5;
      cloud.points[32].z = 0.0;
      cloud.channels[0].values[32] = 199;

      cloud.points[33].x = 1.5;
      cloud.points[33].y = 1.25;
      cloud.points[33].z = 0.0;
      cloud.channels[0].values[33] = 199;

      cloud.points[34].x = 1.5;
      cloud.points[34].y = 1.0;
      cloud.points[34].z = 0.0;
      cloud.channels[0].values[34] = 199;

      cloud.points[35].x = 1.5;
      cloud.points[35].y = 0.75;
      cloud.points[35].z = 0.0;
      cloud.channels[0].values[35] = 199;

      cloud.points[36].x = 1.5;
      cloud.points[36].y = 0.5;
      cloud.points[36].z = 0.0;
      cloud.channels[0].values[36] = 199;

      cloud.points[37].x = 1.5;
      cloud.points[37].y = 0.25;
      cloud.points[37].z = 0.0;
      cloud.channels[0].values[37] = 199;

	// bacs
      cloud.points[38].x = 0.4;
      cloud.points[38].y = 1.7;
      cloud.points[38].z = 0.0;
      cloud.channels[0].values[38] = 199;

      cloud.points[39].x = 0.75;
      cloud.points[39].y = 1.7;
      cloud.points[39].z = 0.0;
      cloud.channels[0].values[39] = 199;

      cloud.points[40].x = 1.1;
      cloud.points[40].y = 1.7;
      cloud.points[40].z = 0.0;
      cloud.channels[0].values[40] = 199;


      cloud.points[41].x = -0.4;
      cloud.points[41].y = 1.7;
      cloud.points[41].z = 0.0;
      cloud.channels[0].values[41] = 199;

      cloud.points[42].x = -0.75;
      cloud.points[42].y = 1.7;
      cloud.points[42].z = 0.0;
      cloud.channels[0].values[42] = 199;

      cloud.points[43].x = -1.1;
      cloud.points[43].y = 1.7;
      cloud.points[43].z = 0.0;
      cloud.channels[0].values[43] = 199;

	// Centre
      cloud.points[44].x = 0.0;
      cloud.points[44].y = 0.950;
      cloud.points[44].z = 0.0;
      cloud.channels[0].values[44] = 199;

      cloud.points[45].x = 0.150;
      cloud.points[45].y = 0.950;
      cloud.points[45].z = 0.0;
      cloud.channels[0].values[45] = 199;

      cloud.points[46].x = -0.150;
      cloud.points[46].y = 0.950;
      cloud.points[46].z = 0.0;
      cloud.channels[0].values[46] = 199;

      cloud.points[47].x = 0.0;
      cloud.points[47].y = 0.800;
      cloud.points[47].z = 0.0;
      cloud.channels[0].values[47] = 199;

      cloud.points[48].x = 0.0;
      cloud.points[48].y = 1.110;
      cloud.points[48].z = 0.0;
      cloud.channels[0].values[48] = 199;

	// Arbres
      cloud.points[49].x = 0.800;
      cloud.points[49].y = 0.05;
      cloud.points[49].z = 0.0;
      cloud.channels[0].values[49] = 199;

      cloud.points[50].x = -0.800;
      cloud.points[50].y = 0.05;
      cloud.points[50].z = 0.0;
      cloud.channels[0].values[50] = 199;

      cloud.points[51].x = 1.450;
      cloud.points[51].y = 0.700;
      cloud.points[51].z = 0.0;
      cloud.channels[0].values[51] = 199;

      cloud.points[52].x = -1.450;
      cloud.points[52].y = 0.700;
      cloud.points[52].z = 0.0;
      cloud.channels[0].values[52] = 199;

	// totem coin
      cloud.points[53].x = 1.345;
      cloud.points[53].y = 0.19;
      cloud.points[53].z = 0.0;
      cloud.channels[0].values[53] = 199;

      cloud.points[54].x = -1.345;
      cloud.points[54].y = 0.19;
      cloud.points[54].z = 0.0;
      cloud.channels[0].values[54] = 199;

/* more precise */

      cloud.points[55].x = 0.125;
      cloud.points[55].y = 0.0;
      cloud.points[55].z = 0.0;
      cloud.channels[0].values[55] = 199;

      cloud.points[56].x = 0.375;
      cloud.points[56].y = 0.0;
      cloud.points[56].z = 0.0;
      cloud.channels[0].values[56] = 199;

      cloud.points[57].x = 0.625;
      cloud.points[57].y = 0.0;
      cloud.points[57].z = 0.0;
      cloud.channels[0].values[57] = 199;

      cloud.points[58].x = 0.875;
      cloud.points[58].y = 0.0;
      cloud.points[58].z = 0.0;
      cloud.channels[0].values[58] = 199;

      cloud.points[59].x = 1.125;
      cloud.points[59].y = 0.0;
      cloud.points[59].z = 0.0;
      cloud.channels[0].values[59] = 199;

      cloud.points[60].x = 1.375;
      cloud.points[60].y = 0.0;
      cloud.points[60].z = 0.0;
      cloud.channels[0].values[60] = 199;

      cloud.points[61].x = 0.125;
      cloud.points[61].y = 2.0;
      cloud.points[61].z = 0.0;
      cloud.channels[0].values[61] = 199;

      cloud.points[62].x = 0.375;
      cloud.points[62].y = 2.0;
      cloud.points[62].z = 0.0;
      cloud.channels[0].values[62] = 199;

      cloud.points[63].x = 0.625;
      cloud.points[63].y = 2.0;
      cloud.points[63].z = 0.0;
      cloud.channels[0].values[63] = 199;

      cloud.points[64].x = 0.875;
      cloud.points[64].y = 2.0;
      cloud.points[64].z = 0.0;
      cloud.channels[0].values[64] = 199;

      cloud.points[65].x = 1.125;
      cloud.points[65].y = 2.0;
      cloud.points[65].z = 0.0;
      cloud.channels[0].values[65] = 199;

      cloud.points[66].x = 1.375;
      cloud.points[66].y = 2.0;
      cloud.points[66].z = 0.0;
      cloud.channels[0].values[66] = 199;

      cloud.points[67].x = 1.5;
      cloud.points[67].y = 1.875;
      cloud.points[67].z = 0.0;
      cloud.channels[0].values[67] = 199;

      cloud.points[68].x = 1.5;
      cloud.points[68].y = 1.625;
      cloud.points[68].z = 0.0;
      cloud.channels[0].values[68] = 199;

      cloud.points[69].x = 1.5;
      cloud.points[69].y = 1.375;
      cloud.points[69].z = 0.0;
      cloud.channels[0].values[69] = 199;

      cloud.points[70].x = 1.5;
      cloud.points[70].y = 1.125;
      cloud.points[70].z = 0.0;
      cloud.channels[0].values[70] = 199;

      cloud.points[71].x = 1.5;
      cloud.points[71].y = 0.875;
      cloud.points[71].z = 0.0;
      cloud.channels[0].values[71] = 199;

      cloud.points[72].x = 1.5;
      cloud.points[72].y = 0.625;
      cloud.points[72].z = 0.0;
      cloud.channels[0].values[72] = 199;

      cloud.points[73].x = 1.5;
      cloud.points[73].y = 0.375;
      cloud.points[73].z = 0.0;
      cloud.channels[0].values[73] = 199;

      cloud.points[74].x = 1.5;
      cloud.points[74].y = 0.125;
      cloud.points[74].z = 0.0;
      cloud.channels[0].values[74] = 199;


      cloud.points[75].x = 0.4;
      cloud.points[75].y = 1.85;
      cloud.points[75].z = 0.0;
      cloud.channels[0].values[75] = 199;

      cloud.points[76].x = 0.575;
      cloud.points[76].y = 1.7;
      cloud.points[76].z = 0.0;
      cloud.channels[0].values[76] = 199;

      cloud.points[77].x = 0.925;
      cloud.points[77].y = 1.7;
      cloud.points[77].z = 0.0;
      cloud.channels[0].values[77] = 199;

      cloud.points[78].x = 1.1;
      cloud.points[78].y = 1.85;
      cloud.points[78].z = 0.0;
      cloud.channels[0].values[78] = 199;



      cloud.points[79].x = -0.125;
      cloud.points[79].y = 0.0;
      cloud.points[79].z = 0.0;
      cloud.channels[0].values[79] = 199;

      cloud.points[80].x = -0.375;
      cloud.points[80].y = 0.0;
      cloud.points[80].z = 0.0;
      cloud.channels[0].values[80] = 199;

      cloud.points[81].x = -0.625;
      cloud.points[81].y = 0.0;
      cloud.points[81].z = 0.0;
      cloud.channels[0].values[81] = 199;

      cloud.points[82].x = -0.875;
      cloud.points[82].y = 0.0;
      cloud.points[82].z = 0.0;
      cloud.channels[0].values[82] = 199;

      cloud.points[83].x = -1.125;
      cloud.points[83].y = 0.0;
      cloud.points[83].z = 0.0;
      cloud.channels[0].values[83] = 199;

      cloud.points[84].x = -1.375;
      cloud.points[84].y = 0.0;
      cloud.points[84].z = 0.0;
      cloud.channels[0].values[84] = 199;

      cloud.points[85].x = -0.125;
      cloud.points[85].y = 2.0;
      cloud.points[85].z = 0.0;
      cloud.channels[0].values[85] = 199;

      cloud.points[86].x = -0.375;
      cloud.points[86].y = 2.0;
      cloud.points[86].z = 0.0;
      cloud.channels[0].values[86] = 199;

      cloud.points[87].x = -0.625;
      cloud.points[87].y = 2.0;
      cloud.points[87].z = 0.0;
      cloud.channels[0].values[87] = 199;

      cloud.points[88].x = -0.875;
      cloud.points[88].y = 2.0;
      cloud.points[88].z = 0.0;
      cloud.channels[0].values[88] = 199;

      cloud.points[89].x = -1.125;
      cloud.points[89].y = 2.0;
      cloud.points[89].z = 0.0;
      cloud.channels[0].values[89] = 199;

      cloud.points[90].x = -1.375;
      cloud.points[90].y = 2.0;
      cloud.points[90].z = 0.0;
      cloud.channels[0].values[90] = 199;

      cloud.points[91].x = -1.5;
      cloud.points[91].y = 1.875;
      cloud.points[91].z = 0.0;
      cloud.channels[0].values[91] = 199;

      cloud.points[92].x = -1.5;
      cloud.points[92].y = 1.625;
      cloud.points[92].z = 0.0;
      cloud.channels[0].values[92] = 199;

      cloud.points[93].x = -1.5;
      cloud.points[93].y = 1.375;
      cloud.points[93].z = 0.0;
      cloud.channels[0].values[93] = 199;

      cloud.points[94].x = -1.5;
      cloud.points[94].y = 1.125;
      cloud.points[94].z = 0.0;
      cloud.channels[0].values[94] = 199;

      cloud.points[95].x = -1.5;
      cloud.points[95].y = 0.875;
      cloud.points[95].z = 0.0;
      cloud.channels[0].values[95] = 199;

      cloud.points[96].x = -1.5;
      cloud.points[96].y = 0.625;
      cloud.points[96].z = 0.0;
      cloud.channels[0].values[96] = 199;

      cloud.points[97].x = -1.5;
      cloud.points[97].y = 0.375;
      cloud.points[97].z = 0.0;
      cloud.channels[0].values[97] = 199;

      cloud.points[98].x = -1.5;
      cloud.points[98].y = 0.125;
      cloud.points[98].z = 0.0;
      cloud.channels[0].values[98] = 199;


      cloud.points[99].x = -0.4;
      cloud.points[99].y = 1.85;
      cloud.points[99].z = 0.0;
      cloud.channels[0].values[99] = 199;

      cloud.points[100].x = -0.575;
      cloud.points[100].y = 1.7;
      cloud.points[100].z = 0.0;
      cloud.channels[0].values[100] = 199;

      cloud.points[101].x = -0.925;
      cloud.points[101].y = 1.7;
      cloud.points[101].z = 0.0;
      cloud.channels[0].values[101] = 199;

      cloud.points[102].x = -1.1;
      cloud.points[102].y = 1.85;
      cloud.points[102].z = 0.0;
      cloud.channels[0].values[102] = 199;




/*
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 1 + count;
      cloud.points[i].y = 2 + count;
      cloud.points[i].z = 3 + count;
      cloud.channels[0].values[i] = 100 + count;
    }
*/
    cloud_pub.publish(cloud);
    ++count;
    r.sleep();
  }
}

