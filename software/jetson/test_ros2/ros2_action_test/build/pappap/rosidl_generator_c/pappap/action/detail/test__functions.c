// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pappap:action/Test.idl
// generated code does not contain a copyright notice
#include "pappap/action/detail/test__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
pappap__action__Test_Goal__init(pappap__action__Test_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // secs
  return true;
}

void
pappap__action__Test_Goal__fini(pappap__action__Test_Goal * msg)
{
  if (!msg) {
    return;
  }
  // secs
}

bool
pappap__action__Test_Goal__are_equal(const pappap__action__Test_Goal * lhs, const pappap__action__Test_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // secs
  if (lhs->secs != rhs->secs) {
    return false;
  }
  return true;
}

bool
pappap__action__Test_Goal__copy(
  const pappap__action__Test_Goal * input,
  pappap__action__Test_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // secs
  output->secs = input->secs;
  return true;
}

pappap__action__Test_Goal *
pappap__action__Test_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Goal * msg = (pappap__action__Test_Goal *)allocator.allocate(sizeof(pappap__action__Test_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_Goal));
  bool success = pappap__action__Test_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_Goal__destroy(pappap__action__Test_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_Goal__Sequence__init(pappap__action__Test_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Goal * data = NULL;

  if (size) {
    data = (pappap__action__Test_Goal *)allocator.zero_allocate(size, sizeof(pappap__action__Test_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_Goal__fini(&data[i - 1]);
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
pappap__action__Test_Goal__Sequence__fini(pappap__action__Test_Goal__Sequence * array)
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
      pappap__action__Test_Goal__fini(&array->data[i]);
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

pappap__action__Test_Goal__Sequence *
pappap__action__Test_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Goal__Sequence * array = (pappap__action__Test_Goal__Sequence *)allocator.allocate(sizeof(pappap__action__Test_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_Goal__Sequence__destroy(pappap__action__Test_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_Goal__Sequence__are_equal(const pappap__action__Test_Goal__Sequence * lhs, const pappap__action__Test_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_Goal__Sequence__copy(
  const pappap__action__Test_Goal__Sequence * input,
  pappap__action__Test_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_Goal * data =
      (pappap__action__Test_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

bool
pappap__action__Test_Result__init(pappap__action__Test_Result * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    pappap__action__Test_Result__fini(msg);
    return false;
  }
  return true;
}

void
pappap__action__Test_Result__fini(pappap__action__Test_Result * msg)
{
  if (!msg) {
    return;
  }
  // status
  rosidl_runtime_c__String__fini(&msg->status);
}

bool
pappap__action__Test_Result__are_equal(const pappap__action__Test_Result * lhs, const pappap__action__Test_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
pappap__action__Test_Result__copy(
  const pappap__action__Test_Result * input,
  pappap__action__Test_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

pappap__action__Test_Result *
pappap__action__Test_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Result * msg = (pappap__action__Test_Result *)allocator.allocate(sizeof(pappap__action__Test_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_Result));
  bool success = pappap__action__Test_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_Result__destroy(pappap__action__Test_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_Result__Sequence__init(pappap__action__Test_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Result * data = NULL;

  if (size) {
    data = (pappap__action__Test_Result *)allocator.zero_allocate(size, sizeof(pappap__action__Test_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_Result__fini(&data[i - 1]);
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
pappap__action__Test_Result__Sequence__fini(pappap__action__Test_Result__Sequence * array)
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
      pappap__action__Test_Result__fini(&array->data[i]);
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

pappap__action__Test_Result__Sequence *
pappap__action__Test_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Result__Sequence * array = (pappap__action__Test_Result__Sequence *)allocator.allocate(sizeof(pappap__action__Test_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_Result__Sequence__destroy(pappap__action__Test_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_Result__Sequence__are_equal(const pappap__action__Test_Result__Sequence * lhs, const pappap__action__Test_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_Result__Sequence__copy(
  const pappap__action__Test_Result__Sequence * input,
  pappap__action__Test_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_Result * data =
      (pappap__action__Test_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `feedback`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
pappap__action__Test_Feedback__init(pappap__action__Test_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__String__init(&msg->feedback)) {
    pappap__action__Test_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
pappap__action__Test_Feedback__fini(pappap__action__Test_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // feedback
  rosidl_runtime_c__String__fini(&msg->feedback);
}

bool
pappap__action__Test_Feedback__are_equal(const pappap__action__Test_Feedback * lhs, const pappap__action__Test_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
pappap__action__Test_Feedback__copy(
  const pappap__action__Test_Feedback * input,
  pappap__action__Test_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__String__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

pappap__action__Test_Feedback *
pappap__action__Test_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Feedback * msg = (pappap__action__Test_Feedback *)allocator.allocate(sizeof(pappap__action__Test_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_Feedback));
  bool success = pappap__action__Test_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_Feedback__destroy(pappap__action__Test_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_Feedback__Sequence__init(pappap__action__Test_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Feedback * data = NULL;

  if (size) {
    data = (pappap__action__Test_Feedback *)allocator.zero_allocate(size, sizeof(pappap__action__Test_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_Feedback__fini(&data[i - 1]);
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
pappap__action__Test_Feedback__Sequence__fini(pappap__action__Test_Feedback__Sequence * array)
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
      pappap__action__Test_Feedback__fini(&array->data[i]);
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

pappap__action__Test_Feedback__Sequence *
pappap__action__Test_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_Feedback__Sequence * array = (pappap__action__Test_Feedback__Sequence *)allocator.allocate(sizeof(pappap__action__Test_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_Feedback__Sequence__destroy(pappap__action__Test_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_Feedback__Sequence__are_equal(const pappap__action__Test_Feedback__Sequence * lhs, const pappap__action__Test_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_Feedback__Sequence__copy(
  const pappap__action__Test_Feedback__Sequence * input,
  pappap__action__Test_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_Feedback * data =
      (pappap__action__Test_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_Feedback__copy(
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
// #include "pappap/action/detail/test__functions.h"

bool
pappap__action__Test_SendGoal_Request__init(pappap__action__Test_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    pappap__action__Test_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!pappap__action__Test_Goal__init(&msg->goal)) {
    pappap__action__Test_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
pappap__action__Test_SendGoal_Request__fini(pappap__action__Test_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  pappap__action__Test_Goal__fini(&msg->goal);
}

bool
pappap__action__Test_SendGoal_Request__are_equal(const pappap__action__Test_SendGoal_Request * lhs, const pappap__action__Test_SendGoal_Request * rhs)
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
  if (!pappap__action__Test_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
pappap__action__Test_SendGoal_Request__copy(
  const pappap__action__Test_SendGoal_Request * input,
  pappap__action__Test_SendGoal_Request * output)
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
  if (!pappap__action__Test_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

pappap__action__Test_SendGoal_Request *
pappap__action__Test_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_SendGoal_Request * msg = (pappap__action__Test_SendGoal_Request *)allocator.allocate(sizeof(pappap__action__Test_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_SendGoal_Request));
  bool success = pappap__action__Test_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_SendGoal_Request__destroy(pappap__action__Test_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_SendGoal_Request__Sequence__init(pappap__action__Test_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_SendGoal_Request * data = NULL;

  if (size) {
    data = (pappap__action__Test_SendGoal_Request *)allocator.zero_allocate(size, sizeof(pappap__action__Test_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_SendGoal_Request__fini(&data[i - 1]);
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
pappap__action__Test_SendGoal_Request__Sequence__fini(pappap__action__Test_SendGoal_Request__Sequence * array)
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
      pappap__action__Test_SendGoal_Request__fini(&array->data[i]);
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

pappap__action__Test_SendGoal_Request__Sequence *
pappap__action__Test_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_SendGoal_Request__Sequence * array = (pappap__action__Test_SendGoal_Request__Sequence *)allocator.allocate(sizeof(pappap__action__Test_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_SendGoal_Request__Sequence__destroy(pappap__action__Test_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_SendGoal_Request__Sequence__are_equal(const pappap__action__Test_SendGoal_Request__Sequence * lhs, const pappap__action__Test_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_SendGoal_Request__Sequence__copy(
  const pappap__action__Test_SendGoal_Request__Sequence * input,
  pappap__action__Test_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_SendGoal_Request * data =
      (pappap__action__Test_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_SendGoal_Request__copy(
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
pappap__action__Test_SendGoal_Response__init(pappap__action__Test_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    pappap__action__Test_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
pappap__action__Test_SendGoal_Response__fini(pappap__action__Test_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
pappap__action__Test_SendGoal_Response__are_equal(const pappap__action__Test_SendGoal_Response * lhs, const pappap__action__Test_SendGoal_Response * rhs)
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
pappap__action__Test_SendGoal_Response__copy(
  const pappap__action__Test_SendGoal_Response * input,
  pappap__action__Test_SendGoal_Response * output)
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

pappap__action__Test_SendGoal_Response *
pappap__action__Test_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_SendGoal_Response * msg = (pappap__action__Test_SendGoal_Response *)allocator.allocate(sizeof(pappap__action__Test_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_SendGoal_Response));
  bool success = pappap__action__Test_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_SendGoal_Response__destroy(pappap__action__Test_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_SendGoal_Response__Sequence__init(pappap__action__Test_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_SendGoal_Response * data = NULL;

  if (size) {
    data = (pappap__action__Test_SendGoal_Response *)allocator.zero_allocate(size, sizeof(pappap__action__Test_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_SendGoal_Response__fini(&data[i - 1]);
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
pappap__action__Test_SendGoal_Response__Sequence__fini(pappap__action__Test_SendGoal_Response__Sequence * array)
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
      pappap__action__Test_SendGoal_Response__fini(&array->data[i]);
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

pappap__action__Test_SendGoal_Response__Sequence *
pappap__action__Test_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_SendGoal_Response__Sequence * array = (pappap__action__Test_SendGoal_Response__Sequence *)allocator.allocate(sizeof(pappap__action__Test_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_SendGoal_Response__Sequence__destroy(pappap__action__Test_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_SendGoal_Response__Sequence__are_equal(const pappap__action__Test_SendGoal_Response__Sequence * lhs, const pappap__action__Test_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_SendGoal_Response__Sequence__copy(
  const pappap__action__Test_SendGoal_Response__Sequence * input,
  pappap__action__Test_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_SendGoal_Response * data =
      (pappap__action__Test_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_SendGoal_Response__copy(
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
pappap__action__Test_GetResult_Request__init(pappap__action__Test_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    pappap__action__Test_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
pappap__action__Test_GetResult_Request__fini(pappap__action__Test_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
pappap__action__Test_GetResult_Request__are_equal(const pappap__action__Test_GetResult_Request * lhs, const pappap__action__Test_GetResult_Request * rhs)
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
pappap__action__Test_GetResult_Request__copy(
  const pappap__action__Test_GetResult_Request * input,
  pappap__action__Test_GetResult_Request * output)
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

pappap__action__Test_GetResult_Request *
pappap__action__Test_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_GetResult_Request * msg = (pappap__action__Test_GetResult_Request *)allocator.allocate(sizeof(pappap__action__Test_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_GetResult_Request));
  bool success = pappap__action__Test_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_GetResult_Request__destroy(pappap__action__Test_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_GetResult_Request__Sequence__init(pappap__action__Test_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_GetResult_Request * data = NULL;

  if (size) {
    data = (pappap__action__Test_GetResult_Request *)allocator.zero_allocate(size, sizeof(pappap__action__Test_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_GetResult_Request__fini(&data[i - 1]);
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
pappap__action__Test_GetResult_Request__Sequence__fini(pappap__action__Test_GetResult_Request__Sequence * array)
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
      pappap__action__Test_GetResult_Request__fini(&array->data[i]);
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

pappap__action__Test_GetResult_Request__Sequence *
pappap__action__Test_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_GetResult_Request__Sequence * array = (pappap__action__Test_GetResult_Request__Sequence *)allocator.allocate(sizeof(pappap__action__Test_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_GetResult_Request__Sequence__destroy(pappap__action__Test_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_GetResult_Request__Sequence__are_equal(const pappap__action__Test_GetResult_Request__Sequence * lhs, const pappap__action__Test_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_GetResult_Request__Sequence__copy(
  const pappap__action__Test_GetResult_Request__Sequence * input,
  pappap__action__Test_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_GetResult_Request * data =
      (pappap__action__Test_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_GetResult_Request__copy(
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
// #include "pappap/action/detail/test__functions.h"

bool
pappap__action__Test_GetResult_Response__init(pappap__action__Test_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!pappap__action__Test_Result__init(&msg->result)) {
    pappap__action__Test_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
pappap__action__Test_GetResult_Response__fini(pappap__action__Test_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  pappap__action__Test_Result__fini(&msg->result);
}

bool
pappap__action__Test_GetResult_Response__are_equal(const pappap__action__Test_GetResult_Response * lhs, const pappap__action__Test_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!pappap__action__Test_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
pappap__action__Test_GetResult_Response__copy(
  const pappap__action__Test_GetResult_Response * input,
  pappap__action__Test_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!pappap__action__Test_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

pappap__action__Test_GetResult_Response *
pappap__action__Test_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_GetResult_Response * msg = (pappap__action__Test_GetResult_Response *)allocator.allocate(sizeof(pappap__action__Test_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_GetResult_Response));
  bool success = pappap__action__Test_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_GetResult_Response__destroy(pappap__action__Test_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_GetResult_Response__Sequence__init(pappap__action__Test_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_GetResult_Response * data = NULL;

  if (size) {
    data = (pappap__action__Test_GetResult_Response *)allocator.zero_allocate(size, sizeof(pappap__action__Test_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_GetResult_Response__fini(&data[i - 1]);
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
pappap__action__Test_GetResult_Response__Sequence__fini(pappap__action__Test_GetResult_Response__Sequence * array)
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
      pappap__action__Test_GetResult_Response__fini(&array->data[i]);
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

pappap__action__Test_GetResult_Response__Sequence *
pappap__action__Test_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_GetResult_Response__Sequence * array = (pappap__action__Test_GetResult_Response__Sequence *)allocator.allocate(sizeof(pappap__action__Test_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_GetResult_Response__Sequence__destroy(pappap__action__Test_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_GetResult_Response__Sequence__are_equal(const pappap__action__Test_GetResult_Response__Sequence * lhs, const pappap__action__Test_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_GetResult_Response__Sequence__copy(
  const pappap__action__Test_GetResult_Response__Sequence * input,
  pappap__action__Test_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_GetResult_Response * data =
      (pappap__action__Test_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_GetResult_Response__copy(
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
// #include "pappap/action/detail/test__functions.h"

bool
pappap__action__Test_FeedbackMessage__init(pappap__action__Test_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    pappap__action__Test_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!pappap__action__Test_Feedback__init(&msg->feedback)) {
    pappap__action__Test_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
pappap__action__Test_FeedbackMessage__fini(pappap__action__Test_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  pappap__action__Test_Feedback__fini(&msg->feedback);
}

bool
pappap__action__Test_FeedbackMessage__are_equal(const pappap__action__Test_FeedbackMessage * lhs, const pappap__action__Test_FeedbackMessage * rhs)
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
  if (!pappap__action__Test_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
pappap__action__Test_FeedbackMessage__copy(
  const pappap__action__Test_FeedbackMessage * input,
  pappap__action__Test_FeedbackMessage * output)
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
  if (!pappap__action__Test_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

pappap__action__Test_FeedbackMessage *
pappap__action__Test_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_FeedbackMessage * msg = (pappap__action__Test_FeedbackMessage *)allocator.allocate(sizeof(pappap__action__Test_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__action__Test_FeedbackMessage));
  bool success = pappap__action__Test_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__action__Test_FeedbackMessage__destroy(pappap__action__Test_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__action__Test_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__action__Test_FeedbackMessage__Sequence__init(pappap__action__Test_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_FeedbackMessage * data = NULL;

  if (size) {
    data = (pappap__action__Test_FeedbackMessage *)allocator.zero_allocate(size, sizeof(pappap__action__Test_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__action__Test_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__action__Test_FeedbackMessage__fini(&data[i - 1]);
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
pappap__action__Test_FeedbackMessage__Sequence__fini(pappap__action__Test_FeedbackMessage__Sequence * array)
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
      pappap__action__Test_FeedbackMessage__fini(&array->data[i]);
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

pappap__action__Test_FeedbackMessage__Sequence *
pappap__action__Test_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__action__Test_FeedbackMessage__Sequence * array = (pappap__action__Test_FeedbackMessage__Sequence *)allocator.allocate(sizeof(pappap__action__Test_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__action__Test_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__action__Test_FeedbackMessage__Sequence__destroy(pappap__action__Test_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__action__Test_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__action__Test_FeedbackMessage__Sequence__are_equal(const pappap__action__Test_FeedbackMessage__Sequence * lhs, const pappap__action__Test_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__action__Test_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__action__Test_FeedbackMessage__Sequence__copy(
  const pappap__action__Test_FeedbackMessage__Sequence * input,
  pappap__action__Test_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__action__Test_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__action__Test_FeedbackMessage * data =
      (pappap__action__Test_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__action__Test_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__action__Test_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__action__Test_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
