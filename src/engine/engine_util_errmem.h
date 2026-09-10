// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_

#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <mujoco/mjexport.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjtype.h>
#include "engine/engine_crossplatform.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef mjPRINTFLIKE
  #if defined(__GNUC__)
    #define mjPRINTFLIKE(n, m) __attribute__((format(printf, n, m)))
  #else
    #define mjPRINTFLIKE(n, m)
  #endif  // __GNUC__
#endif  // mjPRINTFLIKE


//------------------------------ malloc and free ---------------------------------------------------

// allocate memory; byte-align on 8; pad size to multiple of 8
MJAPI void* mju_malloc(size_t size);

// free memory with free() by default
MJAPI void mju_free(void* ptr);

// user memory handlers
MJAPI extern void* (*mju_user_malloc)(size_t);
MJAPI extern void (*mju_user_free)(void*);


//------------------------------ logging configuration and handlers --------------------------------

// set the active log handler, return the previous handler
// if handler is NULL, restore the default handler
MJAPI mjfLogHandler mju_setLogHandler(mjfLogHandler handler);

// setjmp/longjmp without the signal mask: a boundary jump is a plain unwind (setjmp saves the
// mask with a system call on macOS, two orders of magnitude slower than the jump itself)
#if defined(_WIN32) || defined(__EMSCRIPTEN__)
  #define mjSETJMP(env)  setjmp(env)
  #define mjLONGJMP(env) longjmp(env, 1)
#else
  #define mjSETJMP(env)  _setjmp(env)
  #define mjLONGJMP(env) _longjmp(env, 1)
#endif

// error boundary of a pipeline call: the jump target of errors raised within the call, and the
// memory state of the mjData to restore; boundaries form a thread-local chain, innermost first
typedef struct mjBoundary_ {
  jmp_buf env;                     // jump target
  size_t pstack;                   // memory state of the mjData at entry
  size_t pbase;
  size_t parena;
  mjtBool threadlock;
  int callback;                    // callback flag at entry, restored at exit
  struct mjBoundary_* prev;        // enclosing boundary on this thread
} mjBoundary;

// push/pop a boundary on the calling thread's chain; a pushed boundary clears the callback flag,
// a popped one restores it
void mju_pushBoundary(mjBoundary* b);
void mju_popBoundary(mjBoundary* b);

// the same for the pipeline entries: inline in C, where the thread-locals are accessible (a C
// thread-local has no C++ access wrapper), through the functions above in C++
#ifndef __cplusplus
extern mjTHREADLOCAL mjBoundary* mju_boundaryHead;

// a callback has been entered on the calling thread since the innermost boundary was armed:
// the frames between that boundary and the next pipeline entry are foreign, so the entry arms
// its own boundary rather than relying on the enclosing one
extern mjTHREADLOCAL int mju_callbackFlag;

static inline void mji_pushBoundary(mjBoundary* b) {
  b->prev = mju_boundaryHead;
  b->callback = mju_callbackFlag;
  mju_callbackFlag = 0;
  mju_boundaryHead = b;
}
static inline void mji_popBoundary(mjBoundary* b) {
  mju_boundaryHead = b->prev;
  mju_callbackFlag = b->callback;
}

// invoke a callback (or a plugin) from the engine: raise the flag for its duration
#define mjCALLBACK(call)                      \
  {                                           \
    int mjcallback_saved_ = mju_callbackFlag; \
    mju_callbackFlag = 1;                     \
    call;                                     \
    mju_callbackFlag = mjcallback_saved_;     \
  }
#else
static inline void mji_pushBoundary(mjBoundary* b) { mju_pushBoundary(b); }
static inline void mji_popBoundary(mjBoundary* b) { mju_popBoundary(b); }
int mju_callbackFlagValue(void);
#define mju_callbackFlag mju_callbackFlagValue()
#endif

// jump to the innermost boundary on the calling thread, return if there is none
void mju_errorJump(void);

// is a boundary armed on the calling thread
MJAPI int _mjPRIVATE__hasBoundary(void);

// interrupt the pipeline call in progress on the calling thread: unwind to the innermost
// boundary and through the enclosing stages without recording an error; return if no boundary
MJAPI void _mjPRIVATE__interrupt(void);

// is an interrupt pending on the calling thread
MJAPI int _mjPRIVATE__interrupted(void);

// was the last pipeline call on the calling thread interrupted; clears the flag
MJAPI int _mjPRIVATE__takeInterrupt(void);

// kind (mjtStatus) of the error being recovered, or 0; clears it
int mju_takeErrorKind(void);

// is an interrupt pending on the calling thread; clear it
int mju_interrupted(void);
void mju_clearInterrupt(void);

// status of an interrupted call while it unwinds, internal: negative, so that the stages refuse
// and the drivers unwind through the same checks as a pending error without reading anything
// but the status; reset to mjSTATUS_OK by the outermost exit, before the call returns, and never
// observed by user code since no callback runs while a call unwinds
#define mjSTATUS_INTERRUPTED (-1000)

// last error message raised on the calling thread
MJAPI const char* _mjPRIVATE__lastError(void);

// set/get default handler configuration
MJAPI mjLogConfig mju_getLogConfig(void);
MJAPI void mju_setLogConfig(mjLogConfig config);

// clear user handlers; restore default processing
MJAPI void mju_clearHandlers(void);

// legacy error/warning handlers (deprecated: prefer mju_setLogHandler)
MJAPI extern void (*mju_user_error)(const char*);
MJAPI extern void (*mju_user_warning)(const char*);


//------------------------------ public message logging --------------------------------------------

// log a fatal error message, write to logfile and console, pause and exit
MJAPI void mju_error(const char* msg, ...) mjPRINTFLIKE(1, 2);
MJAPI void mju_error_v(const char* msg, va_list args);

// log a warning message, write to logfile and console
MJAPI void mju_warning(const char* msg, ...) mjPRINTFLIKE(1, 2);

// log an info message with optional topic filtering
MJAPI void mju_info(int topic, const char* msg, ...) mjPRINTFLIKE(2, 3);

// dispatch a structured log message to the active handler
MJAPI void mju_message(const mjLogMessage* msg);

// (deprecated) write [datetime, type: message] to MUJOCO_LOG.TXT
MJAPI void mju_writeLog(const char* type, const char* msg);


//------------------------------ internal helpers and macros ---------------------------------------

// set thread-local log handler; return previous thread-local handler
MJAPI mjfLogHandler _mjPRIVATE_setTlsLogHandler(mjfLogHandler handler);

// get the currently active global log handler (read-only, no modification)
MJAPI mjfLogHandler _mjPRIVATE_getGlobalLogHandler(void);

// check whether an info topic is enabled
MJAPI mjtBool mju_isTopicEnabled(int topic);

// strip directory from __FILE__ (cross-platform)
static inline const char* BaseName(const char* path) {
  const char* slash = strrchr(path, '/');
  const char* bslash = strrchr(path, '\\');
  if (slash && bslash) return (slash > bslash ? slash : bslash) + 1;
  if (slash) return slash + 1;
  if (bslash) return bslash + 1;
  return path;
}

// internal macro to emit a structured error with source location and kind; the kind reaches
// the log handler in the message and, inside a pipeline call, mjData.status after the recovery
#define mjERROR_(status_, ...)                                 \
  {                                                            \
    mjLogMessage _msg = {.level = mjLOG_ERROR,                 \
                         .status = (status_),                  \
                         .func = __func__,                     \
                         .file = __FILE__,                     \
                         .line = __LINE__};                    \
    snprintf(_msg.subject, sizeof(_msg.subject), __VA_ARGS__); \
    mju_message(&_msg);                                        \
  }

// an error of unclassified kind; engine code uses one of the three classified macros below
#define mjERROR(...) mjERROR_(mjSTATUS_ERROR, __VA_ARGS__)

// out of memory: the stack, the arena, or a fixed-size buffer is exhausted; a larger memory
// setting or a smaller model may succeed
#define mjERROR_OOM(...) mjERROR_(mjSTATUS_OOM, __VA_ARGS__)

// the call cannot succeed as posed: an argument, the model, or a combination of options is
// invalid or unsupported; the same call will fail again
#define mjERROR_INPUT(...) mjERROR_(mjSTATUS_INPUT, __VA_ARGS__)

// an invariant of the engine is violated: a bug in MuJoCo, not something the caller can fix
#define mjERROR_INTERNAL(...) mjERROR_(mjSTATUS_INTERNAL, __VA_ARGS__)

// internal macro to emit a structured debug trace with fast producer-side topic filtering
#ifndef MJ_DISABLE_DEBUG_TRACING
#define mjDEBUG(_topic, ...)                                   \
  if (mju_isTopicEnabled(_topic)) {                            \
    mjLogMessage _msg = {.level = mjLOG_DEBUG,                 \
                         .topic = _topic,                      \
                         .func = __func__};                    \
    snprintf(_msg.subject, sizeof(_msg.subject), __VA_ARGS__); \
    mju_message(&_msg);                                        \
  }
#else
#define mjDEBUG(_topic, ...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_
