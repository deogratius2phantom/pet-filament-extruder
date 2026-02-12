# Unit Tests

This directory is intended for PlatformIO unit tests.

## Running Tests

```bash
pio test
```

## Writing Tests

Create test files with the naming pattern `test_*.cpp` or `test_*.c`.

Example:

```cpp
#include <unity.h>

void test_example(void) {
    TEST_ASSERT_EQUAL(2, 1 + 1);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_example);
    UNITY_END();
}

void loop() {
}
```