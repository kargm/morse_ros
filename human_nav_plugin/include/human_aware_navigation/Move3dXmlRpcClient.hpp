/*
 * Moved3dXmlRpcClient.h
 *
 *  Created on: Mar 17, 2011
 *      Author: kruset
 */

#ifndef MOVED3DXMLRPCCLIENT_H_
#define MOVED3DXMLRPCCLIENT_H_

#include <human_aware_navigation/Move3DWrapper.h>

namespace human_aware_navigation {

class Moved3dXmlRpcClient {
public:
  Moved3dXmlRpcClient(const char* host, int port);
  virtual ~Moved3dXmlRpcClient();

  int planPath(double x, double y, double az, double gx, double gy, double gaz, Human* humans, int human_no, AbsoluteTrajectory *waypoints, int makeSafePath);

private:
  const char* host;
  int port;


};


}

#endif /* MOVED3DXMLRPCCLIENT_H_ */
