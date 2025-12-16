// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from limo_interfaces:action/Speak.idl
// generated code does not contain a copyright notice

#ifndef LIMO_INTERFACES__ACTION__DETAIL__SPEAK__FUNCTIONS_H_
#define LIMO_INTERFACES__ACTION__DETAIL__SPEAK__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "limo_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "limo_interfaces/action/detail/speak__struct.h"

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_Goal
 * )) before or use
 * limo_interfaces__action__Speak_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Goal__init(limo_interfaces__action__Speak_Goal * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Goal__fini(limo_interfaces__action__Speak_Goal * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_Goal *
limo_interfaces__action__Speak_Goal__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Goal__destroy(limo_interfaces__action__Speak_Goal * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Goal__are_equal(const limo_interfaces__action__Speak_Goal * lhs, const limo_interfaces__action__Speak_Goal * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Goal__copy(
  const limo_interfaces__action__Speak_Goal * input,
  limo_interfaces__action__Speak_Goal * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Goal__Sequence__init(limo_interfaces__action__Speak_Goal__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Goal__Sequence__fini(limo_interfaces__action__Speak_Goal__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_Goal__Sequence *
limo_interfaces__action__Speak_Goal__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Goal__Sequence__destroy(limo_interfaces__action__Speak_Goal__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Goal__Sequence__are_equal(const limo_interfaces__action__Speak_Goal__Sequence * lhs, const limo_interfaces__action__Speak_Goal__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Goal__Sequence__copy(
  const limo_interfaces__action__Speak_Goal__Sequence * input,
  limo_interfaces__action__Speak_Goal__Sequence * output);

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_Result
 * )) before or use
 * limo_interfaces__action__Speak_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Result__init(limo_interfaces__action__Speak_Result * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Result__fini(limo_interfaces__action__Speak_Result * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_Result *
limo_interfaces__action__Speak_Result__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Result__destroy(limo_interfaces__action__Speak_Result * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Result__are_equal(const limo_interfaces__action__Speak_Result * lhs, const limo_interfaces__action__Speak_Result * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Result__copy(
  const limo_interfaces__action__Speak_Result * input,
  limo_interfaces__action__Speak_Result * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Result__Sequence__init(limo_interfaces__action__Speak_Result__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Result__Sequence__fini(limo_interfaces__action__Speak_Result__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_Result__Sequence *
limo_interfaces__action__Speak_Result__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Result__Sequence__destroy(limo_interfaces__action__Speak_Result__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Result__Sequence__are_equal(const limo_interfaces__action__Speak_Result__Sequence * lhs, const limo_interfaces__action__Speak_Result__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Result__Sequence__copy(
  const limo_interfaces__action__Speak_Result__Sequence * input,
  limo_interfaces__action__Speak_Result__Sequence * output);

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_Feedback
 * )) before or use
 * limo_interfaces__action__Speak_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Feedback__init(limo_interfaces__action__Speak_Feedback * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Feedback__fini(limo_interfaces__action__Speak_Feedback * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_Feedback *
limo_interfaces__action__Speak_Feedback__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Feedback__destroy(limo_interfaces__action__Speak_Feedback * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Feedback__are_equal(const limo_interfaces__action__Speak_Feedback * lhs, const limo_interfaces__action__Speak_Feedback * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Feedback__copy(
  const limo_interfaces__action__Speak_Feedback * input,
  limo_interfaces__action__Speak_Feedback * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Feedback__Sequence__init(limo_interfaces__action__Speak_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Feedback__Sequence__fini(limo_interfaces__action__Speak_Feedback__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_Feedback__Sequence *
limo_interfaces__action__Speak_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_Feedback__Sequence__destroy(limo_interfaces__action__Speak_Feedback__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Feedback__Sequence__are_equal(const limo_interfaces__action__Speak_Feedback__Sequence * lhs, const limo_interfaces__action__Speak_Feedback__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_Feedback__Sequence__copy(
  const limo_interfaces__action__Speak_Feedback__Sequence * input,
  limo_interfaces__action__Speak_Feedback__Sequence * output);

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_SendGoal_Request
 * )) before or use
 * limo_interfaces__action__Speak_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Request__init(limo_interfaces__action__Speak_SendGoal_Request * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Request__fini(limo_interfaces__action__Speak_SendGoal_Request * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_SendGoal_Request *
limo_interfaces__action__Speak_SendGoal_Request__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Request__destroy(limo_interfaces__action__Speak_SendGoal_Request * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Request__are_equal(const limo_interfaces__action__Speak_SendGoal_Request * lhs, const limo_interfaces__action__Speak_SendGoal_Request * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Request__copy(
  const limo_interfaces__action__Speak_SendGoal_Request * input,
  limo_interfaces__action__Speak_SendGoal_Request * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Request__Sequence__init(limo_interfaces__action__Speak_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Request__Sequence__fini(limo_interfaces__action__Speak_SendGoal_Request__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_SendGoal_Request__Sequence *
limo_interfaces__action__Speak_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Request__Sequence__destroy(limo_interfaces__action__Speak_SendGoal_Request__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Request__Sequence__are_equal(const limo_interfaces__action__Speak_SendGoal_Request__Sequence * lhs, const limo_interfaces__action__Speak_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Request__Sequence__copy(
  const limo_interfaces__action__Speak_SendGoal_Request__Sequence * input,
  limo_interfaces__action__Speak_SendGoal_Request__Sequence * output);

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_SendGoal_Response
 * )) before or use
 * limo_interfaces__action__Speak_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Response__init(limo_interfaces__action__Speak_SendGoal_Response * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Response__fini(limo_interfaces__action__Speak_SendGoal_Response * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_SendGoal_Response *
limo_interfaces__action__Speak_SendGoal_Response__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Response__destroy(limo_interfaces__action__Speak_SendGoal_Response * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Response__are_equal(const limo_interfaces__action__Speak_SendGoal_Response * lhs, const limo_interfaces__action__Speak_SendGoal_Response * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Response__copy(
  const limo_interfaces__action__Speak_SendGoal_Response * input,
  limo_interfaces__action__Speak_SendGoal_Response * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Response__Sequence__init(limo_interfaces__action__Speak_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Response__Sequence__fini(limo_interfaces__action__Speak_SendGoal_Response__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_SendGoal_Response__Sequence *
limo_interfaces__action__Speak_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_SendGoal_Response__Sequence__destroy(limo_interfaces__action__Speak_SendGoal_Response__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Response__Sequence__are_equal(const limo_interfaces__action__Speak_SendGoal_Response__Sequence * lhs, const limo_interfaces__action__Speak_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_SendGoal_Response__Sequence__copy(
  const limo_interfaces__action__Speak_SendGoal_Response__Sequence * input,
  limo_interfaces__action__Speak_SendGoal_Response__Sequence * output);

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_GetResult_Request
 * )) before or use
 * limo_interfaces__action__Speak_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Request__init(limo_interfaces__action__Speak_GetResult_Request * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Request__fini(limo_interfaces__action__Speak_GetResult_Request * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_GetResult_Request *
limo_interfaces__action__Speak_GetResult_Request__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Request__destroy(limo_interfaces__action__Speak_GetResult_Request * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Request__are_equal(const limo_interfaces__action__Speak_GetResult_Request * lhs, const limo_interfaces__action__Speak_GetResult_Request * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Request__copy(
  const limo_interfaces__action__Speak_GetResult_Request * input,
  limo_interfaces__action__Speak_GetResult_Request * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Request__Sequence__init(limo_interfaces__action__Speak_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Request__Sequence__fini(limo_interfaces__action__Speak_GetResult_Request__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_GetResult_Request__Sequence *
limo_interfaces__action__Speak_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Request__Sequence__destroy(limo_interfaces__action__Speak_GetResult_Request__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Request__Sequence__are_equal(const limo_interfaces__action__Speak_GetResult_Request__Sequence * lhs, const limo_interfaces__action__Speak_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Request__Sequence__copy(
  const limo_interfaces__action__Speak_GetResult_Request__Sequence * input,
  limo_interfaces__action__Speak_GetResult_Request__Sequence * output);

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_GetResult_Response
 * )) before or use
 * limo_interfaces__action__Speak_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Response__init(limo_interfaces__action__Speak_GetResult_Response * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Response__fini(limo_interfaces__action__Speak_GetResult_Response * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_GetResult_Response *
limo_interfaces__action__Speak_GetResult_Response__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Response__destroy(limo_interfaces__action__Speak_GetResult_Response * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Response__are_equal(const limo_interfaces__action__Speak_GetResult_Response * lhs, const limo_interfaces__action__Speak_GetResult_Response * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Response__copy(
  const limo_interfaces__action__Speak_GetResult_Response * input,
  limo_interfaces__action__Speak_GetResult_Response * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Response__Sequence__init(limo_interfaces__action__Speak_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Response__Sequence__fini(limo_interfaces__action__Speak_GetResult_Response__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_GetResult_Response__Sequence *
limo_interfaces__action__Speak_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_GetResult_Response__Sequence__destroy(limo_interfaces__action__Speak_GetResult_Response__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Response__Sequence__are_equal(const limo_interfaces__action__Speak_GetResult_Response__Sequence * lhs, const limo_interfaces__action__Speak_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_GetResult_Response__Sequence__copy(
  const limo_interfaces__action__Speak_GetResult_Response__Sequence * input,
  limo_interfaces__action__Speak_GetResult_Response__Sequence * output);

/// Initialize action/Speak message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * limo_interfaces__action__Speak_FeedbackMessage
 * )) before or use
 * limo_interfaces__action__Speak_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_FeedbackMessage__init(limo_interfaces__action__Speak_FeedbackMessage * msg);

/// Finalize action/Speak message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_FeedbackMessage__fini(limo_interfaces__action__Speak_FeedbackMessage * msg);

/// Create action/Speak message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * limo_interfaces__action__Speak_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_FeedbackMessage *
limo_interfaces__action__Speak_FeedbackMessage__create();

/// Destroy action/Speak message.
/**
 * It calls
 * limo_interfaces__action__Speak_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_FeedbackMessage__destroy(limo_interfaces__action__Speak_FeedbackMessage * msg);

/// Check for action/Speak message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_FeedbackMessage__are_equal(const limo_interfaces__action__Speak_FeedbackMessage * lhs, const limo_interfaces__action__Speak_FeedbackMessage * rhs);

/// Copy a action/Speak message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_FeedbackMessage__copy(
  const limo_interfaces__action__Speak_FeedbackMessage * input,
  limo_interfaces__action__Speak_FeedbackMessage * output);

/// Initialize array of action/Speak messages.
/**
 * It allocates the memory for the number of elements and calls
 * limo_interfaces__action__Speak_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_FeedbackMessage__Sequence__init(limo_interfaces__action__Speak_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_FeedbackMessage__Sequence__fini(limo_interfaces__action__Speak_FeedbackMessage__Sequence * array);

/// Create array of action/Speak messages.
/**
 * It allocates the memory for the array and calls
 * limo_interfaces__action__Speak_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
limo_interfaces__action__Speak_FeedbackMessage__Sequence *
limo_interfaces__action__Speak_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Speak messages.
/**
 * It calls
 * limo_interfaces__action__Speak_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
void
limo_interfaces__action__Speak_FeedbackMessage__Sequence__destroy(limo_interfaces__action__Speak_FeedbackMessage__Sequence * array);

/// Check for action/Speak message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_FeedbackMessage__Sequence__are_equal(const limo_interfaces__action__Speak_FeedbackMessage__Sequence * lhs, const limo_interfaces__action__Speak_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Speak messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_limo_interfaces
bool
limo_interfaces__action__Speak_FeedbackMessage__Sequence__copy(
  const limo_interfaces__action__Speak_FeedbackMessage__Sequence * input,
  limo_interfaces__action__Speak_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LIMO_INTERFACES__ACTION__DETAIL__SPEAK__FUNCTIONS_H_
