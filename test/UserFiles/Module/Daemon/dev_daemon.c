/**
*   @file dev_daemon.c
*   @brief 守护进程(看门狗)软件实现源文件
*   @author HuaCheng Ma
*   @date 2026/1/16
*   @version 0.1
*   @note 该功能需要FreeRTOS才可使用
*/
#include "dev_daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"



Daemon_Instance_s* Daemon_Register(const Daemon_InitConfig_s* config)
{
    if (config == NULL){
        // Log_Error("Daemon config is empty!");
        return NULL;
    }
    Daemon_Instance_s* instance = (Daemon_Instance_s*)user_malloc(sizeof(Daemon_Instance_s));
    if (instance == NULL){
        // Log_Error("Daemon instance malloc failed!");
        return NULL;
    }
    instance->daemon_callback = config->daemon_callback;
    instance->daemon_reload_callback = config->daemon_reload_callback;
    instance->is_started = false;
    instance->is_timeout = false;
    instance->timeout_ms = config->timeout_ms;
    instance->last_feed_time = 0;
    return instance;
}

void Daemon_Start(Daemon_Instance_s* instance)
{
    if (instance == NULL){
        // Log_Error("Daemon instance is NULL!");
        return;
    }
    if (instance->is_started)
    {
        // Log_Warning("Daemon instance is already started!");
        return;
    }
    instance->is_started = true;
    instance->is_timeout = false;
    TickType_t tick = xTaskGetTickCount();
    instance->last_feed_time = tick * 1000 / configTICK_RATE_HZ;
}

void Daemon_Feed(Daemon_Instance_s* instance)
{
    if (instance == NULL){
        // Log_Error("Daemon instance is NULL!");
        return;
    }
    if (!instance->is_started){
        // Log_Warning("Daemon instance hasn't started yet!");
        return;
    }
    if (instance->is_timeout){
        //如果在已超时状态，则重置看门狗
        // Log_Warning("Daemon instance is timeout!");
        Daemon_Reload(instance);
        return;
    }
    TickType_t tick = xTaskGetTickCount();
    instance->last_feed_time = tick * 1000 / configTICK_RATE_HZ;
}


void Daemon_Reload(Daemon_Instance_s* instance)
{
    if (instance == NULL){
        // Log_Error("Daemon instance is NULL!");
        return;
    }
    instance->is_started = true;
    instance->is_timeout = false;
    if (instance->daemon_reload_callback!=NULL) {
        instance->daemon_reload_callback(instance);
    }
    Daemon_Feed(instance);
}


bool Daemon_Step(Daemon_Instance_s* instance)
{
    if (instance == NULL){
        // Log_Error("Daemon instance is NULL!");
        return false;
    }
    if (!instance->is_started){
        // Log_Warning("Daemon instance hasn't started yet!");
        return false;
    }
    if (instance->is_timeout){
        // Log_Warning("Daemon instance is timeout!");
        return false;
    }
    TickType_t tick = xTaskGetTickCount()* 1000 / configTICK_RATE_HZ;
    if (tick-instance->last_feed_time >= instance->timeout_ms)
    {
        instance->is_timeout = true;
        if (instance->daemon_callback!=NULL) {
            instance->daemon_callback(instance);
        }
        return false;
    }
    return true;
}