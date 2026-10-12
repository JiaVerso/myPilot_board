#ifndef H743_MCN_H
#define H743_MCN_H

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCN_MAX_TOPICS       24U
#define MCN_MAX_SUBSCRIBERS  8U
#define MCN_MAX_TOPIC_NAME   32U
#define MCN_MAX_DATA_SIZE    256U

typedef struct
{
    char name[MCN_MAX_TOPIC_NAME];                          /* Topic 名称 */
    rt_uint16_t data_size;
    rt_uint32_t sequence;
    rt_uint8_t data[MCN_MAX_DATA_SIZE];
    mcn_subscriber_t *subscribers[MCN_MAX_SUBSCRIBERS];     /* 订阅该 Topic 的订阅者指针数组 */
    rt_bool_t used;                                         /* Topic 是否已分配 */
} mcn_topic_t;


typedef struct
{
    mcn_topic_t *topic;                                     /* 指向订阅的 Topic */
    struct rt_semaphore semaphore;
    rt_uint32_t last_sequence;
    rt_bool_t active;
} mcn_subscriber_t;

enum {
    MCN_OK       = 0,
    MCN_EINVAL   = -1,
    MCN_ENOMEM   = -2,
    MCN_ENOENT   = -3,
    MCN_EBUSY    = -4,
    MCN_ETIMEOUT = -5,
    MCN_ENODATA  = -6,
};

typedef struct mcn_subscriber *mcn_sub_t;

int mcn_init(void);
mcn_topic_t *mcn_advertise(const char *name, uint16_t data_size)
mcn_sub_t mcn_subscribe(const char *name, rt_bool_t start_with_latest);
int mcn_publish(mcn_topic_t topic, const void *data);
int mcn_copy(mcn_sub_t sub, void *data, rt_bool_t only_if_updated);
int mcn_wait(mcn_sub_t sub, rt_int32_t timeout, void *data);
int mcn_unsubscribe(mcn_sub_t sub);
uint32_t mcn_sequence(mcn_sub_t sub);

#ifdef __cplusplus
}
#endif
#endif
