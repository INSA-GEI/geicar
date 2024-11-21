/*
 * application.cpp
 *
 *  Created on: Oct 23, 2024
 *      Author: dimercur
 */

#include "application.h"

namespace app {

Application *application;

Application::Application() {
	// TODO Auto-generated constructor stub

}

Application::~Application() {
	// TODO Auto-generated destructor stub
}

void APPLICATION_Init() {
  application = new Application();
}

} /* namespace app */



