#include "uMcn.h"

#include <rtthread.h>
#include <string.h>

#define MCN_SUBSCRIBER_COUNT \
    (MCN_MAX_TOPICS * MCN_MAX_SUBSCRIBERS)

 /* 全局静态资源分配 */
static mcn_topic_t mcn_topics[MCN_MAX_TOPICS];
static mcn_subscriber_t mcn_subscribers[MCN_SUBSCRIBER_COUNT];

static struct rt_mutex mcn_mutex;
static rt_bool_t mcn_initialized;

 /* take lock */ 
static int mcn_lock(void)
{
    if (!mcn_initialized)
    {
        return MCN_EBUSY;
    }

    if (rt_mutex_take(&mcn_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return MCN_EBUSY;
    }

    return MCN_OK;
}

 /* release lock */ 
static void mcn_unlock(void)
{
    rt_mutex_release(&mcn_mutex);
}

static rt_bool_t mcn_topic_name_valid(const char *name)
{
    if (name == RT_NULL || name[0] == '\0')
    {
        return RT_FALSE;
    }

    return strlen(name) < MCN_MAX_TOPIC_NAME;
}

 /* 查找话题名称 */ 
static mcn_topic_t *mcn_find_topic(const char *name)
{
    rt_uint32_t index;

    for (index = 0; index < MCN_MAX_TOPICS; index++)
    {
        if (mcn_topics[index].used &&
            strncmp(mcn_topics[index].name,
                    name,
                    MCN_MAX_TOPIC_NAME) == 0)
        {
            return &mcn_topics[index];
        }
    }

    return RT_NULL;
}

 /* 查找空闲订阅者 */ 
static mcn_subscriber_t *mcn_find_free_subscriber(rt_uint32_t *subscriber_index)
{
    rt_uint32_t index;

    for (index = 0; index < MCN_SUBSCRIBER_COUNT; index++)
    {
        if (!mcn_subscribers[index].active)
        {
            if (subscriber_index != RT_NULL)
            {
                *subscriber_index = index;
            }

            return &mcn_subscribers[index];
        }
    }

    return RT_NULL;
}

 /* initialize mcn module*/
int mcn_init(void)
{
    rt_err_t result;

    if (mcn_initialized)
    {
        return MCN_OK;
    }

    memset(mcn_topics, 0, sizeof(mcn_topics));
    memset(mcn_subscribers, 0, sizeof(mcn_subscribers));

    result = rt_mutex_init(&mcn_mutex,
                           "mcn",
                           RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        return MCN_EBUSY;
    }

    mcn_initialized = RT_TRUE;
    return MCN_OK;
}

 /* 创建话题 */ 
mcn_topic_t *mcn_advertise(const char *name, uint16_t data_size)
{
    rt_uint32_t index;
    mcn_topic_t *topic;

    /* check */
    if (!mcn_initialized ||
        !mcn_topic_name_valid(name) ||
        data_size == 0 ||
        data_size > MCN_MAX_DATA_SIZE)
    {
        return RT_NULL;
    }

    if (mcn_lock() != MCN_OK)
    {
        return RT_NULL;
    }

    topic = mcn_find_topic(name);
    if (topic != RT_NULL)
    {
        if (topic->data_size != data_size)
        {
            topic = RT_NULL;
        }

        mcn_unlock();
        return topic;
    }

    /* 遍历空闲topic数组 */ 
    for (index = 0; index < MCN_MAX_TOPICS; index++)
    {
        if (!mcn_topics[index].used)
        {
            topic = &mcn_topics[index];
            memset(topic, 0, sizeof(*topic));
            strncpy(topic->name, name, MCN_MAX_TOPIC_NAME - 1u);
            topic->data_size = data_size;
            topic->used = RT_TRUE;

            mcn_unlock();
            return topic;
        }
    }

    mcn_unlock();
    return RT_NULL;
}

 /* subscribe topic */
mcn_sub_t mcn_subscribe(const char *name, rt_bool_t start_with_latest)
{
    rt_uint32_t index;
    rt_uint32_t subscriber_index;
    rt_err_t result;

    char semaphore_name[RT_NAME_MAX];
    mcn_topic_t *topic;
    mcn_subscriber_t *subscriber;

    if (!mcn_initialized || !mcn_topic_name_valid(name))
    {
        return RT_NULL;
    }

    if (mcn_lock() != MCN_OK)
    {
        return RT_NULL;
    }

    /* 判断话题是否存在 */
    topic = mcn_find_topic(name);
    if (topic == RT_NULL)
    {
        mcn_unlock();
        return RT_NULL;
    }

    subscriber = mcn_find_free_subscriber(&subscriber_index);
    if (subscriber == RT_NULL)
    {
        mcn_unlock();
        return RT_NULL;
    }

    for (index = 0; index < MCN_MAX_SUBSCRIBERS; index++)
    {
        if (topic->subscribers[index] == RT_NULL)
        {
            break;
        }
    }

    /* 话题订阅对象溢出 */ 
    if (index == MCN_MAX_SUBSCRIBERS)
    {
        mcn_unlock();
        return RT_NULL;
    }

    result = rt_sem_init(&subscriber->semaphore,
                         "mcn_semaphore",
                         0,
                         RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        mcn_unlock();
        return RT_NULL;
    }

    subscriber->topic = topic;
    subscriber->last_sequence = start_with_latest ? topic->sequence : 0u;
    subscriber->active = RT_TRUE;
    topic->subscribers[index] = subscriber;

    mcn_unlock();
    return subscriber;
}

int mcn_publish(mcn_topic_t *topic, const void *data)
{
    rt_uint32_t index;

    if (topic == RT_NULL || data == RT_NULL || !topic->used)
    {
        return MCN_EINVAL;
    }

    if (mcn_lock() != MCN_OK)
    {
        return MCN_EBUSY;
    }

    /* 从data源地址开始，拷贝size大小的内存区域到dest */
    memcpy(topic->data, data, topic->data_size);
    topic->sequence++;

    for (index = 0; index < MCN_MAX_SUBSCRIBERS; index++)
    {
        if (topic->subscribers[index] != RT_NULL)
        {
            /* 唤醒订阅对象 */
            rt_sem_release(&topic->subscribers[index]->semaphore);
        }
    }

    mcn_unlock();
    return MCN_OK;
}

 /* 复制数据到订阅对象缓冲区 */
int mcn_copy(mcn_subscriber_t *subscriber, const void *data, rt_bool_t only_if_updated)
{
    mcn_topic_t *topic;

    if (subscriber == RT_NULL || data == RT_NULL || !subscriber->active)
    {
        return MCN_EINVAL;
    }

    if (mcn_lock() != MCN_OK)
    {
        return MCN_EBUSY;
    }

    topic = subscriber->topic;
    if (topic == RT_NULL || !subscriber->active)
    {
        mcn_unlock();
        return MCN_EINVAL;
    }

    if (only_if_updated && topic->sequence == subscriber->last_sequence)
    {
        mcn_unlock();
        return MCN_ENODATA;
    }

    memcpy(data, topic->data, topic->data_size);
    subscriber->last_sequence = topic->sequence;

    mcn_unlock();
    return MCN_OK;
}

 /* 等待新数据到达 */
int mcn_wait(mcn_subscriber_t *subscriber, rt_int32_t timeout, void *data)
{
    int result;
    rt_err_t semaphore_result;

    if (subscriber == RT_NULL || data == RT_NULL || !subscriber->active)
    {
        return MCN_EINVAL;
    }

    result = mcn_copy(subscriber, data, RT_TRUE);
    if (result == MCN_OK)
    {
        return MCN_OK;
    }

    if (result != MCN_ENODATA)
    {
        return result;
    }

    semaphore_result = rt_sem_take(&subscriber->semaphore, timeout);
    if (semaphore_result != RT_EOK)
    {
        return semaphore_result == -RT_ETIMEOUT ?
            MCN_ETIMEOUT : MCN_EBUSY;
    }

    return mcn_copy(subscriber, data, RT_TRUE);
}

 /* 取消订阅 */
int mcn_unsubscribe(mcn_subscriber_t *subscriber)
{
    rt_uint32_t index;
    mcn_topic_t *topic;

    if (subscriber == RT_NULL || !subscriber->active)
    {
        return MCN_EINVAL;
    }

    if (mcn_lock() != MCN_OK)
    {
        return MCN_EBUSY;
    }

    topic = subscriber->topic;
    if (topic == RT_NULL)
    {
        mcn_unlock();
        return MCN_EINVAL;
    }

    for (index = 0; index < MCN_MAX_SUBSCRIBERS; index++)
    {
        if (topic->subscribers[index] == subscriber)
        {
            topic->subscribers[index] = RT_NULL;
        }
    }

    subscriber->active = RT_FALSE;
    subscriber->topic = RT_NULL;
    subscriber->last_sequence = 0u;
    rt_sem_detach(&subscriber->semaphore);

    mcn_unlock();
    return MCN_OK;
}

 /* 获取订阅者的序列号 */
uint32_t mcn_sequence(mcn_subscriber_t *subscriber)
{
    rt_uint32_t sequence;

    if (subscriber == RT_NULL || !subscriber->active)
    {
        return 0u;
    }

    if (mcn_lock() != MCN_OK)
    {
        return 0u;
    }

    if (subscriber->topic == RT_NULL || !subscriber->active)
    {
        mcn_unlock();
        return 0u;
    }

    sequence = subscriber->topic->sequence;
    mcn_unlock();
    return sequence;
}
