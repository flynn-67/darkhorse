// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from limo_interfaces:action/Speak.idl
// generated code does not contain a copyright notice
#include "limo_interfaces/action/detail/speak__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `text`
#include "rosidl_runtime_c/string_functions.h"

bool
limo_interfaces__action__Speak_Goal__init(limo_interfaces__action__Speak_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    limo_interfaces__action__Speak_Goal__fini(msg);
    return false;
  }
  return true;
}

void
limo_interfaces__action__Speak_Goal__fini(limo_interfaces__action__Speak_Goal * msg)
{
  if (!msg) {
    return;
  }
  // text
  rosidl_runtime_c__String__fini(&msg->text);
}

bool
limo_interfaces__action__Speak_Goal__are_equal(const limo_interfaces__action__Speak_Goal * lhs, const limo_interfaces__action__Speak_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_Goal__copy(
  const limo_interfaces__action__Speak_Goal * input,
  limo_interfaces__action__Speak_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  return true;
}

limo_interfaces__action__Speak_Goal *
limo_interfaces__action__Speak_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Goal * msg = (limo_interfaces__action__Speak_Goal *)allocator.allocate(sizeof(limo_interfaces__action__Speak_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_Goal));
  bool success = limo_interfaces__action__Speak_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_Goal__destroy(limo_interfaces__action__Speak_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_Goal__Sequence__init(limo_interfaces__action__Speak_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Goal * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_Goal *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_Goal__Sequence__fini(limo_interfaces__action__Speak_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_Goal__Sequence *
limo_interfaces__action__Speak_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Goal__Sequence * array = (limo_interfaces__action__Speak_Goal__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_Goal__Sequence__destroy(limo_interfaces__action__Speak_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_Goal__Sequence__are_equal(const limo_interfaces__action__Speak_Goal__Sequence * lhs, const limo_interfaces__action__Speak_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_Goal__Sequence__copy(
  const limo_interfaces__action__Speak_Goal__Sequence * input,
  limo_interfaces__action__Speak_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_Goal * data =
      (limo_interfaces__action__Speak_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
limo_interfaces__action__Speak_Result__init(limo_interfaces__action__Speak_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
limo_interfaces__action__Speak_Result__fini(limo_interfaces__action__Speak_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
limo_interfaces__action__Speak_Result__are_equal(const limo_interfaces__action__Speak_Result * lhs, const limo_interfaces__action__Speak_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_Result__copy(
  const limo_interfaces__action__Speak_Result * input,
  limo_interfaces__action__Speak_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

limo_interfaces__action__Speak_Result *
limo_interfaces__action__Speak_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Result * msg = (limo_interfaces__action__Speak_Result *)allocator.allocate(sizeof(limo_interfaces__action__Speak_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_Result));
  bool success = limo_interfaces__action__Speak_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_Result__destroy(limo_interfaces__action__Speak_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_Result__Sequence__init(limo_interfaces__action__Speak_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Result * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_Result *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_Result__Sequence__fini(limo_interfaces__action__Speak_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_Result__Sequence *
limo_interfaces__action__Speak_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Result__Sequence * array = (limo_interfaces__action__Speak_Result__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_Result__Sequence__destroy(limo_interfaces__action__Speak_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_Result__Sequence__are_equal(const limo_interfaces__action__Speak_Result__Sequence * lhs, const limo_interfaces__action__Speak_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_Result__Sequence__copy(
  const limo_interfaces__action__Speak_Result__Sequence * input,
  limo_interfaces__action__Speak_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_Result * data =
      (limo_interfaces__action__Speak_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
limo_interfaces__action__Speak_Feedback__init(limo_interfaces__action__Speak_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // progress
  return true;
}

void
limo_interfaces__action__Speak_Feedback__fini(limo_interfaces__action__Speak_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // progress
}

bool
limo_interfaces__action__Speak_Feedback__are_equal(const limo_interfaces__action__Speak_Feedback * lhs, const limo_interfaces__action__Speak_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // progress
  if (lhs->progress != rhs->progress) {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_Feedback__copy(
  const limo_interfaces__action__Speak_Feedback * input,
  limo_interfaces__action__Speak_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // progress
  output->progress = input->progress;
  return true;
}

limo_interfaces__action__Speak_Feedback *
limo_interfaces__action__Speak_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Feedback * msg = (limo_interfaces__action__Speak_Feedback *)allocator.allocate(sizeof(limo_interfaces__action__Speak_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_Feedback));
  bool success = limo_interfaces__action__Speak_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_Feedback__destroy(limo_interfaces__action__Speak_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_Feedback__Sequence__init(limo_interfaces__action__Speak_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Feedback * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_Feedback *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_Feedback__Sequence__fini(limo_interfaces__action__Speak_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_Feedback__Sequence *
limo_interfaces__action__Speak_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_Feedback__Sequence * array = (limo_interfaces__action__Speak_Feedback__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_Feedback__Sequence__destroy(limo_interfaces__action__Speak_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_Feedback__Sequence__are_equal(const limo_interfaces__action__Speak_Feedback__Sequence * lhs, const limo_interfaces__action__Speak_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_Feedback__Sequence__copy(
  const limo_interfaces__action__Speak_Feedback__Sequence * input,
  limo_interfaces__action__Speak_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_Feedback * data =
      (limo_interfaces__action__Speak_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "limo_interfaces/action/detail/speak__functions.h"

bool
limo_interfaces__action__Speak_SendGoal_Request__init(limo_interfaces__action__Speak_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    limo_interfaces__action__Speak_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!limo_interfaces__action__Speak_Goal__init(&msg->goal)) {
    limo_interfaces__action__Speak_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
limo_interfaces__action__Speak_SendGoal_Request__fini(limo_interfaces__action__Speak_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  limo_interfaces__action__Speak_Goal__fini(&msg->goal);
}

bool
limo_interfaces__action__Speak_SendGoal_Request__are_equal(const limo_interfaces__action__Speak_SendGoal_Request * lhs, const limo_interfaces__action__Speak_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!limo_interfaces__action__Speak_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_SendGoal_Request__copy(
  const limo_interfaces__action__Speak_SendGoal_Request * input,
  limo_interfaces__action__Speak_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!limo_interfaces__action__Speak_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

limo_interfaces__action__Speak_SendGoal_Request *
limo_interfaces__action__Speak_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_SendGoal_Request * msg = (limo_interfaces__action__Speak_SendGoal_Request *)allocator.allocate(sizeof(limo_interfaces__action__Speak_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_SendGoal_Request));
  bool success = limo_interfaces__action__Speak_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_SendGoal_Request__destroy(limo_interfaces__action__Speak_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_SendGoal_Request__Sequence__init(limo_interfaces__action__Speak_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_SendGoal_Request * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_SendGoal_Request *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_SendGoal_Request__Sequence__fini(limo_interfaces__action__Speak_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_SendGoal_Request__Sequence *
limo_interfaces__action__Speak_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_SendGoal_Request__Sequence * array = (limo_interfaces__action__Speak_SendGoal_Request__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_SendGoal_Request__Sequence__destroy(limo_interfaces__action__Speak_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_SendGoal_Request__Sequence__are_equal(const limo_interfaces__action__Speak_SendGoal_Request__Sequence * lhs, const limo_interfaces__action__Speak_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_SendGoal_Request__Sequence__copy(
  const limo_interfaces__action__Speak_SendGoal_Request__Sequence * input,
  limo_interfaces__action__Speak_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_SendGoal_Request * data =
      (limo_interfaces__action__Speak_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
limo_interfaces__action__Speak_SendGoal_Response__init(limo_interfaces__action__Speak_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    limo_interfaces__action__Speak_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
limo_interfaces__action__Speak_SendGoal_Response__fini(limo_interfaces__action__Speak_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
limo_interfaces__action__Speak_SendGoal_Response__are_equal(const limo_interfaces__action__Speak_SendGoal_Response * lhs, const limo_interfaces__action__Speak_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_SendGoal_Response__copy(
  const limo_interfaces__action__Speak_SendGoal_Response * input,
  limo_interfaces__action__Speak_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

limo_interfaces__action__Speak_SendGoal_Response *
limo_interfaces__action__Speak_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_SendGoal_Response * msg = (limo_interfaces__action__Speak_SendGoal_Response *)allocator.allocate(sizeof(limo_interfaces__action__Speak_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_SendGoal_Response));
  bool success = limo_interfaces__action__Speak_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_SendGoal_Response__destroy(limo_interfaces__action__Speak_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_SendGoal_Response__Sequence__init(limo_interfaces__action__Speak_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_SendGoal_Response * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_SendGoal_Response *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_SendGoal_Response__Sequence__fini(limo_interfaces__action__Speak_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_SendGoal_Response__Sequence *
limo_interfaces__action__Speak_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_SendGoal_Response__Sequence * array = (limo_interfaces__action__Speak_SendGoal_Response__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_SendGoal_Response__Sequence__destroy(limo_interfaces__action__Speak_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_SendGoal_Response__Sequence__are_equal(const limo_interfaces__action__Speak_SendGoal_Response__Sequence * lhs, const limo_interfaces__action__Speak_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_SendGoal_Response__Sequence__copy(
  const limo_interfaces__action__Speak_SendGoal_Response__Sequence * input,
  limo_interfaces__action__Speak_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_SendGoal_Response * data =
      (limo_interfaces__action__Speak_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
limo_interfaces__action__Speak_GetResult_Request__init(limo_interfaces__action__Speak_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    limo_interfaces__action__Speak_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
limo_interfaces__action__Speak_GetResult_Request__fini(limo_interfaces__action__Speak_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
limo_interfaces__action__Speak_GetResult_Request__are_equal(const limo_interfaces__action__Speak_GetResult_Request * lhs, const limo_interfaces__action__Speak_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_GetResult_Request__copy(
  const limo_interfaces__action__Speak_GetResult_Request * input,
  limo_interfaces__action__Speak_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

limo_interfaces__action__Speak_GetResult_Request *
limo_interfaces__action__Speak_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_GetResult_Request * msg = (limo_interfaces__action__Speak_GetResult_Request *)allocator.allocate(sizeof(limo_interfaces__action__Speak_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_GetResult_Request));
  bool success = limo_interfaces__action__Speak_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_GetResult_Request__destroy(limo_interfaces__action__Speak_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_GetResult_Request__Sequence__init(limo_interfaces__action__Speak_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_GetResult_Request * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_GetResult_Request *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_GetResult_Request__Sequence__fini(limo_interfaces__action__Speak_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_GetResult_Request__Sequence *
limo_interfaces__action__Speak_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_GetResult_Request__Sequence * array = (limo_interfaces__action__Speak_GetResult_Request__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_GetResult_Request__Sequence__destroy(limo_interfaces__action__Speak_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_GetResult_Request__Sequence__are_equal(const limo_interfaces__action__Speak_GetResult_Request__Sequence * lhs, const limo_interfaces__action__Speak_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_GetResult_Request__Sequence__copy(
  const limo_interfaces__action__Speak_GetResult_Request__Sequence * input,
  limo_interfaces__action__Speak_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_GetResult_Request * data =
      (limo_interfaces__action__Speak_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "limo_interfaces/action/detail/speak__functions.h"

bool
limo_interfaces__action__Speak_GetResult_Response__init(limo_interfaces__action__Speak_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!limo_interfaces__action__Speak_Result__init(&msg->result)) {
    limo_interfaces__action__Speak_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
limo_interfaces__action__Speak_GetResult_Response__fini(limo_interfaces__action__Speak_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  limo_interfaces__action__Speak_Result__fini(&msg->result);
}

bool
limo_interfaces__action__Speak_GetResult_Response__are_equal(const limo_interfaces__action__Speak_GetResult_Response * lhs, const limo_interfaces__action__Speak_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!limo_interfaces__action__Speak_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_GetResult_Response__copy(
  const limo_interfaces__action__Speak_GetResult_Response * input,
  limo_interfaces__action__Speak_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!limo_interfaces__action__Speak_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

limo_interfaces__action__Speak_GetResult_Response *
limo_interfaces__action__Speak_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_GetResult_Response * msg = (limo_interfaces__action__Speak_GetResult_Response *)allocator.allocate(sizeof(limo_interfaces__action__Speak_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_GetResult_Response));
  bool success = limo_interfaces__action__Speak_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_GetResult_Response__destroy(limo_interfaces__action__Speak_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_GetResult_Response__Sequence__init(limo_interfaces__action__Speak_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_GetResult_Response * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_GetResult_Response *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_GetResult_Response__Sequence__fini(limo_interfaces__action__Speak_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_GetResult_Response__Sequence *
limo_interfaces__action__Speak_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_GetResult_Response__Sequence * array = (limo_interfaces__action__Speak_GetResult_Response__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_GetResult_Response__Sequence__destroy(limo_interfaces__action__Speak_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_GetResult_Response__Sequence__are_equal(const limo_interfaces__action__Speak_GetResult_Response__Sequence * lhs, const limo_interfaces__action__Speak_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_GetResult_Response__Sequence__copy(
  const limo_interfaces__action__Speak_GetResult_Response__Sequence * input,
  limo_interfaces__action__Speak_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_GetResult_Response * data =
      (limo_interfaces__action__Speak_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "limo_interfaces/action/detail/speak__functions.h"

bool
limo_interfaces__action__Speak_FeedbackMessage__init(limo_interfaces__action__Speak_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    limo_interfaces__action__Speak_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!limo_interfaces__action__Speak_Feedback__init(&msg->feedback)) {
    limo_interfaces__action__Speak_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
limo_interfaces__action__Speak_FeedbackMessage__fini(limo_interfaces__action__Speak_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  limo_interfaces__action__Speak_Feedback__fini(&msg->feedback);
}

bool
limo_interfaces__action__Speak_FeedbackMessage__are_equal(const limo_interfaces__action__Speak_FeedbackMessage * lhs, const limo_interfaces__action__Speak_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!limo_interfaces__action__Speak_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
limo_interfaces__action__Speak_FeedbackMessage__copy(
  const limo_interfaces__action__Speak_FeedbackMessage * input,
  limo_interfaces__action__Speak_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!limo_interfaces__action__Speak_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

limo_interfaces__action__Speak_FeedbackMessage *
limo_interfaces__action__Speak_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_FeedbackMessage * msg = (limo_interfaces__action__Speak_FeedbackMessage *)allocator.allocate(sizeof(limo_interfaces__action__Speak_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(limo_interfaces__action__Speak_FeedbackMessage));
  bool success = limo_interfaces__action__Speak_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
limo_interfaces__action__Speak_FeedbackMessage__destroy(limo_interfaces__action__Speak_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    limo_interfaces__action__Speak_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
limo_interfaces__action__Speak_FeedbackMessage__Sequence__init(limo_interfaces__action__Speak_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_FeedbackMessage * data = NULL;

  if (size) {
    data = (limo_interfaces__action__Speak_FeedbackMessage *)allocator.zero_allocate(size, sizeof(limo_interfaces__action__Speak_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = limo_interfaces__action__Speak_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        limo_interfaces__action__Speak_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
limo_interfaces__action__Speak_FeedbackMessage__Sequence__fini(limo_interfaces__action__Speak_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      limo_interfaces__action__Speak_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

limo_interfaces__action__Speak_FeedbackMessage__Sequence *
limo_interfaces__action__Speak_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  limo_interfaces__action__Speak_FeedbackMessage__Sequence * array = (limo_interfaces__action__Speak_FeedbackMessage__Sequence *)allocator.allocate(sizeof(limo_interfaces__action__Speak_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = limo_interfaces__action__Speak_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
limo_interfaces__action__Speak_FeedbackMessage__Sequence__destroy(limo_interfaces__action__Speak_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    limo_interfaces__action__Speak_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
limo_interfaces__action__Speak_FeedbackMessage__Sequence__are_equal(const limo_interfaces__action__Speak_FeedbackMessage__Sequence * lhs, const limo_interfaces__action__Speak_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!limo_interfaces__action__Speak_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
limo_interfaces__action__Speak_FeedbackMessage__Sequence__copy(
  const limo_interfaces__action__Speak_FeedbackMessage__Sequence * input,
  limo_interfaces__action__Speak_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(limo_interfaces__action__Speak_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    limo_interfaces__action__Speak_FeedbackMessage * data =
      (limo_interfaces__action__Speak_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!limo_interfaces__action__Speak_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          limo_interfaces__action__Speak_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!limo_interfaces__action__Speak_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
