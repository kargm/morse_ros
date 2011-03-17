/*
 * Moved3dXmlRpcClient.cpp
 *
 *  Created on: Mar 17, 2011
 *      Author: kruset
 */

#include <human_aware_navigation/Move3dXmlRpcClient.hpp>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "XmlRpc.h"

using namespace XmlRpc;

namespace human_aware_navigation {


Moved3dXmlRpcClient::Moved3dXmlRpcClient(const char* host, int port) {
  XmlRpc::setVerbosity(5);
  this->host = host;
  this->port = port;
}

Moved3dXmlRpcClient::~Moved3dXmlRpcClient() {
}

 int Moved3dXmlRpcClient::planPath(double x, double y, double az, double gx, double gy, double gaz, Human* humans, int human_no, AbsoluteTrajectory *waypoints, int makeSafePath) {
   XmlRpcClient client(host, port);
   XmlRpcValue params;

   std::cout << "plan Path from "<< x<< ", "<<y<<" to "<<gx<<", "<<gy<<"\n";

   params["x"] = x;
   params["y"] = y;
   params["az"] = az;
   params["gx"] = gx;
   params["gy"] = gy;
   params["gaz"] = gaz;

   XmlRpcValue humanarr;
   humanarr.setSize(MHPD_MAX_HUMANS);
   for (int human_i = 0; human_i < MHPD_MAX_HUMANS ; human_i++) {
     XmlRpcValue humanstruct;
     if (human_i < human_no && humans[human_i].exists) {
       humanstruct["x"] = humans[human_i].x;
       humanstruct["y"] = humans[human_i].y;
       humanstruct["az"] = humans[human_i].az;
       humanstruct["pose"] = humans[human_i].pose;
       humanstruct["locked"] = humans[human_i].locked;
     }
     humanarr[human_i] = humanstruct;
   }
   params["humans_array"] = humanarr;
   XmlRpcValue booleanFalse(false);
   params["makeSafe"] = booleanFalse;

   XmlRpcValue result;

   if (client.execute(MXR_REQUEST_PATH_METHOD_NAME, params, result)) {
     std::cout << "\nMethods:\n " << params << "\n\n";
   } else {
     std::cout << "Error calling"<<MXR_REQUEST_PATH_METHOD_NAME<<" \n\n";
   }

   return 0;
 }


}
