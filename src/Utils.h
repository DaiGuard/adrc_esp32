#ifndef __UTILS_H__
#define __UTILS_H__

#define RUNTIME_SOFT_CHECK(fn, s, ...) { \
    bool ret = fn; \
    if(!ret) \
    { \
        log_warn(s, ##__VA_ARGS__); \
    } \
}

#define RUNTIME_CHECK(fn, s, ...) { \
    bool ret = fn; \
    while(!ret) \
    { \
        log_erro(s, ##__VA_ARGS__); \
        delay(500); \
    } \
}


#define RCL_CHECK(fn, s) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(s);}}
#define RCL_SOFT_CHECK(fn, s) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_one(s);}}


void error_one(const char* msg)
{
    Serial.println(msg);    
}
void error_loop(const char* msg) {
  while(1) {
    Serial.println(msg);
    delay(100);
  }
}

#endif