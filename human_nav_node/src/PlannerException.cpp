/*
 * PlannerException.cpp
 *
 *  Created on: Mar 20, 2011
 *      Author: kruset
 */

#include <iostream>
#include <string>
#include <exception>
using namespace std;

class PlannerException: public exception
{
  string message;
  PlannerException(string newMessage) {
    message = newMessage;
  }

  virtual const char* what() const throw()
  {
    return message;
  }
} PlannerException;


