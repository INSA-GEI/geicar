/*
 * application.h
 *
 *  Created on: Oct 23, 2024
 *      Author: dimercur
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C
#endif

namespace app {

class Application {
public:
	Application();
	virtual ~Application();

private:

};

} /* namespace app */

//typedef void* mylibrary_mytype_t;

EXTERN_C void APPLICATION_Init();
//EXTERN_C void mylibrary_mytype_destroy(mylibrary_mytype_t mytype);
//EXTERN_C void mylibrary_mytype_doit(mylibrary_mytype_t self, int param);

#undef EXTERNC
#endif /* APPLICATION_H_ */
