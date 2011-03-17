/*
 * Moved3dXmlRpcClient.cpp
 *
 *  Created on: Mar 17, 2011
 *      Author: kruset
 */

#include <human_aware_navigation/Moved3dXmlRpcClient.h>

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

 int Moved3dXmlRpcClient::planPath() {
   XmlRpcClient client(host, port);
   XmlRpcValue params;

   std::cout << "planPath\n";

   return 0;
 }


}
