// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#ifndef _LocalizationTest_Msg_h
#define _LocalizationTest_Msg_h

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _LocalizationTest_Msg LocalizationTest_Msg;
struct _LocalizationTest_Msg
{
    int8_t     version;
    int8_t     RealSensorCount;
    float      position[3];
    float      vel[3];
    int16_t    *dis;
};

/**
 * Create a deep copy of a LocalizationTest_Msg.
 * When no longer needed, destroy it with LocalizationTest_Msg_destroy()
 */
LocalizationTest_Msg* LocalizationTest_Msg_copy(const LocalizationTest_Msg* to_copy);

/**
 * Destroy an instance of LocalizationTest_Msg created by LocalizationTest_Msg_copy()
 */
void LocalizationTest_Msg_destroy(LocalizationTest_Msg* to_destroy);

/**
 * Identifies a single subscription.  This is an opaque data type.
 */
typedef struct _LocalizationTest_Msg_subscription_t LocalizationTest_Msg_subscription_t;

/**
 * Prototype for a callback function invoked when a message of type
 * LocalizationTest_Msg is received.
 */
typedef void(*LocalizationTest_Msg_handler_t)(
    const lcm_recv_buf_t *rbuf, const char *channel,
    const LocalizationTest_Msg *msg, void *userdata);

/**
 * Publish a message of type LocalizationTest_Msg using LCM.
 *
 * @param lcm The LCM instance to publish with.
 * @param channel The channel to publish on.
 * @param msg The message to publish.
 * @return 0 on success, <0 on error.  Success means LCM has transferred
 * responsibility of the message data to the OS.
 */
int LocalizationTest_Msg_publish(lcm_t *lcm, const char *channel, const LocalizationTest_Msg *msg);

/**
 * Subscribe to messages of type LocalizationTest_Msg using LCM.
 *
 * @param lcm The LCM instance to subscribe with.
 * @param channel The channel to subscribe to.
 * @param handler The callback function invoked by LCM when a message is
 *     received. This function is invoked by LCM during calls to lcm_handle()
 *     and lcm_handle_timeout().
 * @param userdata An opaque pointer passed to @p handler when it is invoked.
 * @return 0 on success, <0 if an error occured
 */
LocalizationTest_Msg_subscription_t* LocalizationTest_Msg_subscribe(
    lcm_t *lcm, const char *channel, LocalizationTest_Msg_handler_t handler, void *userdata);

/**
 * Removes and destroys a subscription created by LocalizationTest_Msg_subscribe()
 */
int LocalizationTest_Msg_unsubscribe(lcm_t *lcm, LocalizationTest_Msg_subscription_t* hid);

/**
 * Sets the queue capacity for a subscription.
 * Some LCM providers (e.g., the default multicast provider) are implemented
 * using a background receive thread that constantly revceives messages from
 * the network.  As these messages are received, they are buffered on
 * per-subscription queues until dispatched by lcm_handle().  This function
 * how many messages are queued before dropping messages.
 *
 * @param subs the subscription to modify.
 * @param num_messages The maximum number of messages to queue
 *  on the subscription.
 * @return 0 on success, <0 if an error occured
 */
int LocalizationTest_Msg_subscription_set_queue_capacity(
    LocalizationTest_Msg_subscription_t* subs, int num_messages);

/**
 * Encode a message of type LocalizationTest_Msg into binary form.
 *
 * @param buf The output buffer.
 * @param offset Encoding starts at this byte offset into @p buf.
 * @param maxlen Maximum number of bytes to write.  This should generally
 *               be equal to LocalizationTest_Msg_encoded_size().
 * @param msg The message to encode.
 * @return The number of bytes encoded, or <0 if an error occured.
 */
int LocalizationTest_Msg_encode(void *buf, int offset, int maxlen, const LocalizationTest_Msg *p);

/**
 * Decode a message of type LocalizationTest_Msg from binary form.
 * When decoding messages containing strings or variable-length arrays, this
 * function may allocate memory.  When finished with the decoded message,
 * release allocated resources with LocalizationTest_Msg_decode_cleanup().
 *
 * @param buf The buffer containing the encoded message
 * @param offset The byte offset into @p buf where the encoded message starts.
 * @param maxlen The maximum number of bytes to read while decoding.
 * @param msg Output parameter where the decoded message is stored
 * @return The number of bytes decoded, or <0 if an error occured.
 */
int LocalizationTest_Msg_decode(const void *buf, int offset, int maxlen, LocalizationTest_Msg *msg);

/**
 * Release resources allocated by LocalizationTest_Msg_decode()
 * @return 0
 */
int LocalizationTest_Msg_decode_cleanup(LocalizationTest_Msg *p);

/**
 * Check how many bytes are required to encode a message of type LocalizationTest_Msg
 */
int LocalizationTest_Msg_encoded_size(const LocalizationTest_Msg *p);

// LCM support functions. Users should not call these
int64_t __LocalizationTest_Msg_get_hash(void);
uint64_t __LocalizationTest_Msg_hash_recursive(const __lcm_hash_ptr *p);
int __LocalizationTest_Msg_encode_array(
    void *buf, int offset, int maxlen, const LocalizationTest_Msg *p, int elements);
int __LocalizationTest_Msg_decode_array(
    const void *buf, int offset, int maxlen, LocalizationTest_Msg *p, int elements);
int __LocalizationTest_Msg_decode_array_cleanup(LocalizationTest_Msg *p, int elements);
int __LocalizationTest_Msg_encoded_array_size(const LocalizationTest_Msg *p, int elements);
int __LocalizationTest_Msg_clone_array(const LocalizationTest_Msg *p, LocalizationTest_Msg *q, int elements);

#ifdef __cplusplus
}
#endif

#endif