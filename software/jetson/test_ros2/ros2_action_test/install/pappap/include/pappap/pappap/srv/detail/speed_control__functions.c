// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pappap:srv/SpeedControl.idl
// generated code does not contain a copyright notice
#include "pappap/srv/detail/speed_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
pappap__srv__SpeedControl_Request__init(pappap__srv__SpeedControl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // spd
  return true;
}

void
pappap__srv__SpeedControl_Request__fini(pappap__srv__SpeedControl_Request * msg)
{
  if (!msg) {
    return;
  }
  // spd
}

bool
pappap__srv__SpeedControl_Request__are_equal(const pappap__srv__SpeedControl_Request * lhs, const pappap__srv__SpeedControl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // spd
  if (lhs->spd != rhs->spd) {
    return false;
  }
  return true;
}

bool
pappap__srv__SpeedControl_Request__copy(
  const pappap__srv__SpeedControl_Request * input,
  pappap__srv__SpeedControl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // spd
  output->spd = input->spd;
  return true;
}

pappap__srv__SpeedControl_Request *
pappap__srv__SpeedControl_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__srv__SpeedControl_Request * msg = (pappap__srv__SpeedControl_Request *)allocator.allocate(sizeof(pappap__srv__SpeedControl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__srv__SpeedControl_Request));
  bool success = pappap__srv__SpeedControl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__srv__SpeedControl_Request__destroy(pappap__srv__SpeedControl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__srv__SpeedControl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__srv__SpeedControl_Request__Sequence__init(pappap__srv__SpeedControl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__srv__SpeedControl_Request * data = NULL;

  if (size) {
    data = (pappap__srv__SpeedControl_Request *)allocator.zero_allocate(size, sizeof(pappap__srv__SpeedControl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__srv__SpeedControl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__srv__SpeedControl_Request__fini(&data[i - 1]);
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
pappap__srv__SpeedControl_Request__Sequence__fini(pappap__srv__SpeedControl_Request__Sequence * array)
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
      pappap__srv__SpeedControl_Request__fini(&array->data[i]);
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

pappap__srv__SpeedControl_Request__Sequence *
pappap__srv__SpeedControl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__srv__SpeedControl_Request__Sequence * array = (pappap__srv__SpeedControl_Request__Sequence *)allocator.allocate(sizeof(pappap__srv__SpeedControl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__srv__SpeedControl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__srv__SpeedControl_Request__Sequence__destroy(pappap__srv__SpeedControl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__srv__SpeedControl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__srv__SpeedControl_Request__Sequence__are_equal(const pappap__srv__SpeedControl_Request__Sequence * lhs, const pappap__srv__SpeedControl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__srv__SpeedControl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__srv__SpeedControl_Request__Sequence__copy(
  const pappap__srv__SpeedControl_Request__Sequence * input,
  pappap__srv__SpeedControl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__srv__SpeedControl_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__srv__SpeedControl_Request * data =
      (pappap__srv__SpeedControl_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__srv__SpeedControl_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__srv__SpeedControl_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__srv__SpeedControl_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `feedback`
#include "rosidl_runtime_c/string_functions.h"

bool
pappap__srv__SpeedControl_Response__init(pappap__srv__SpeedControl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__String__init(&msg->feedback)) {
    pappap__srv__SpeedControl_Response__fini(msg);
    return false;
  }
  return true;
}

void
pappap__srv__SpeedControl_Response__fini(pappap__srv__SpeedControl_Response * msg)
{
  if (!msg) {
    return;
  }
  // feedback
  rosidl_runtime_c__String__fini(&msg->feedback);
}

bool
pappap__srv__SpeedControl_Response__are_equal(const pappap__srv__SpeedControl_Response * lhs, const pappap__srv__SpeedControl_Response * rhs)
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
pappap__srv__SpeedControl_Response__copy(
  const pappap__srv__SpeedControl_Response * input,
  pappap__srv__SpeedControl_Response * output)
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

pappap__srv__SpeedControl_Response *
pappap__srv__SpeedControl_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__srv__SpeedControl_Response * msg = (pappap__srv__SpeedControl_Response *)allocator.allocate(sizeof(pappap__srv__SpeedControl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pappap__srv__SpeedControl_Response));
  bool success = pappap__srv__SpeedControl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pappap__srv__SpeedControl_Response__destroy(pappap__srv__SpeedControl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pappap__srv__SpeedControl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pappap__srv__SpeedControl_Response__Sequence__init(pappap__srv__SpeedControl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__srv__SpeedControl_Response * data = NULL;

  if (size) {
    data = (pappap__srv__SpeedControl_Response *)allocator.zero_allocate(size, sizeof(pappap__srv__SpeedControl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pappap__srv__SpeedControl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pappap__srv__SpeedControl_Response__fini(&data[i - 1]);
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
pappap__srv__SpeedControl_Response__Sequence__fini(pappap__srv__SpeedControl_Response__Sequence * array)
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
      pappap__srv__SpeedControl_Response__fini(&array->data[i]);
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

pappap__srv__SpeedControl_Response__Sequence *
pappap__srv__SpeedControl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pappap__srv__SpeedControl_Response__Sequence * array = (pappap__srv__SpeedControl_Response__Sequence *)allocator.allocate(sizeof(pappap__srv__SpeedControl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pappap__srv__SpeedControl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pappap__srv__SpeedControl_Response__Sequence__destroy(pappap__srv__SpeedControl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pappap__srv__SpeedControl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pappap__srv__SpeedControl_Response__Sequence__are_equal(const pappap__srv__SpeedControl_Response__Sequence * lhs, const pappap__srv__SpeedControl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pappap__srv__SpeedControl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pappap__srv__SpeedControl_Response__Sequence__copy(
  const pappap__srv__SpeedControl_Response__Sequence * input,
  pappap__srv__SpeedControl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pappap__srv__SpeedControl_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pappap__srv__SpeedControl_Response * data =
      (pappap__srv__SpeedControl_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pappap__srv__SpeedControl_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pappap__srv__SpeedControl_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pappap__srv__SpeedControl_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
