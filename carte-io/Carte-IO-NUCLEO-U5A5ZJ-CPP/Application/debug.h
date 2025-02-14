/*
 * debug.h
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#define DEBUG_DEFAULT_PORT	0  // le port SWO par defaut pour le debug
#define DEBUG_VCP_PORT		32 // le port VCP correspond à l'uart connecté à la sonde de debug

#ifdef __cplusplus

#include "taskhandler.h"

#define PANIC(msg) Debug::panic(__FILE__, __LINE__, msg)

class Debug {
public:
	// Wrapper statique pour appeler la méthode membre
	Debug();

	static void write(uint8_t port, const char c) { write_(port, c); }
	static void write(const char c) { write_(DEBUG_DEFAULT_PORT, c); }

	static void write(uint8_t port, const char *str) { write_(port, str); }
	static void writeln(uint8_t port, const char *str) { write_(port, str); write_(port, "\n"); }
	static void write(const char *str) { write_(DEBUG_DEFAULT_PORT, str); }
	static void writeln(const char *str) { writeln(DEBUG_DEFAULT_PORT, str); }

	static void write(uint8_t port, const char* fmt, ...);
	static void write(const char* fmt, ...);

	static void panic(const char* file, uint32_t line, const char* msg);
private:
	TaskHandler periodicReportTaskHandler_;
	void periodicReportTask_(void);

	static void write_(uint8_t port, const char c);
	static void write_(uint8_t port, const char *str);
};
#endif /* __cplusplus */

int __io_putchar(int ch);
int __io_getchar(void);

void panic (const char *file, uint32_t line, const char *msg);
#endif /* DEBUG_H_ */
