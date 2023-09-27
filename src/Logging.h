#ifndef __LOGGING_H__
#define __LOGGING_H__

#define MAX_LOG_STRING 128

// #define log_base(m, s, ...) do{ }while(0)
#define log_base(m, s, ...) do{ \
        if(Serial.availableForWrite() > 32) \
        { \
            char text[MAX_LOG_STRING]; \
            sprintf(text, s, ##__VA_ARGS__); \
            Serial.print(m); Serial.println(text); \
        } \
    } while(0)

#if VERBOSE_LEVEL > 0
    #define log_erro(s, ...) log_base("E: ", s, ##__VA_ARGS__)
#else
    #define log_erro(s, ...)
#endif

#if VERBOSE_LEVEL > 1
    #define log_warn(s, ...) log_base("W: ", s, ##__VA_ARGS__)
#else
    #define log_warn(s, ...)
#endif

#if VERBOSE_LEVEL > 2
    #define log_info(s, ...) log_base("I: ", s, ##__VA_ARGS__)
#else
    #define log_info(s, ...)
#endif

#if VERBOSE_LEVEL > 3
    #define log_debug(s, ...) log_base("D: ", s, ##__VA_ARGS__)
#else
    #define log_debug(s, ...)
#endif


#endif