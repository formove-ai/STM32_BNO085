/*
 * @file cmock_strict_order.c
 * @brief Definitions for CMock's strict call-ordering counters.
 *
 * cmock_config.yml enables :enforce_strict_ordering, which makes the generated
 * mock reference these two counters as externs. Ceedling emits them from its
 * generated test runner; builds that supply their own runner, such as
 * PlatformIO's, have to define them somewhere instead. Shipping them here keeps
 * consumers from having to know that.
 *
 * MockBNO085_SPI_Library_Destroy() resets both, and _Init() calls _Destroy(),
 * so zero initialisation is all that is required.
 */

int GlobalExpectCount;
int GlobalVerifyOrder;
