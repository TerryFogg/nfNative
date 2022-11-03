@echo off
REM Run this file to build the project outside of the IDE.
REM WARNING: if using a different machine, copy the .rsp files together with this script.
echo txe_block_allocate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_block_allocate.gcc.rsp" || exit 1
echo txe_block_pool_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_block_pool_create.gcc.rsp" || exit 1
echo txe_block_pool_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_block_pool_delete.gcc.rsp" || exit 1
echo txe_block_pool_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_block_pool_info_get.gcc.rsp" || exit 1
echo txe_block_pool_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_block_pool_prioritize.gcc.rsp" || exit 1
echo txe_block_release.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_block_release.gcc.rsp" || exit 1
echo txe_byte_allocate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_byte_allocate.gcc.rsp" || exit 1
echo txe_byte_pool_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_byte_pool_create.gcc.rsp" || exit 1
echo txe_byte_pool_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_byte_pool_delete.gcc.rsp" || exit 1
echo txe_byte_pool_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_byte_pool_info_get.gcc.rsp" || exit 1
echo txe_byte_pool_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_byte_pool_prioritize.gcc.rsp" || exit 1
echo txe_byte_release.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_byte_release.gcc.rsp" || exit 1
echo txe_event_flags_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_event_flags_create.gcc.rsp" || exit 1
echo txe_event_flags_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_event_flags_delete.gcc.rsp" || exit 1
echo txe_event_flags_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_event_flags_get.gcc.rsp" || exit 1
echo txe_event_flags_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_event_flags_info_get.gcc.rsp" || exit 1
echo txe_event_flags_set.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_event_flags_set.gcc.rsp" || exit 1
echo txe_event_flags_set_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_event_flags_set_notify.gcc.rsp" || exit 1
echo txe_mutex_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_mutex_create.gcc.rsp" || exit 1
echo txe_mutex_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_mutex_delete.gcc.rsp" || exit 1
echo txe_mutex_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_mutex_get.gcc.rsp" || exit 1
echo txe_mutex_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_mutex_info_get.gcc.rsp" || exit 1
echo txe_mutex_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_mutex_prioritize.gcc.rsp" || exit 1
echo txe_mutex_put.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_mutex_put.gcc.rsp" || exit 1
echo txe_queue_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_create.gcc.rsp" || exit 1
echo txe_queue_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_delete.gcc.rsp" || exit 1
echo txe_queue_flush.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_flush.gcc.rsp" || exit 1
echo txe_queue_front_send.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_front_send.gcc.rsp" || exit 1
echo txe_queue_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_info_get.gcc.rsp" || exit 1
echo txe_queue_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_prioritize.gcc.rsp" || exit 1
echo txe_queue_receive.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_receive.gcc.rsp" || exit 1
echo txe_queue_send.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_send.gcc.rsp" || exit 1
echo txe_queue_send_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_queue_send_notify.gcc.rsp" || exit 1
echo txe_semaphore_ceiling_put.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_ceiling_put.gcc.rsp" || exit 1
echo txe_semaphore_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_create.gcc.rsp" || exit 1
echo txe_semaphore_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_delete.gcc.rsp" || exit 1
echo txe_semaphore_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_get.gcc.rsp" || exit 1
echo txe_semaphore_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_info_get.gcc.rsp" || exit 1
echo txe_semaphore_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_prioritize.gcc.rsp" || exit 1
echo txe_semaphore_put.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_put.gcc.rsp" || exit 1
echo txe_semaphore_put_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_semaphore_put_notify.gcc.rsp" || exit 1
echo txe_thread_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_create.gcc.rsp" || exit 1
echo txe_thread_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_delete.gcc.rsp" || exit 1
echo txe_thread_entry_exit_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_entry_exit_notify.gcc.rsp" || exit 1
echo txe_thread_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_info_get.gcc.rsp" || exit 1
echo txe_thread_preemption_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_preemption_change.gcc.rsp" || exit 1
echo txe_thread_priority_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_priority_change.gcc.rsp" || exit 1
echo txe_thread_relinquish.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_relinquish.gcc.rsp" || exit 1
echo txe_thread_reset.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_reset.gcc.rsp" || exit 1
echo txe_thread_resume.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_resume.gcc.rsp" || exit 1
echo txe_thread_suspend.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_suspend.gcc.rsp" || exit 1
echo txe_thread_terminate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_terminate.gcc.rsp" || exit 1
echo txe_thread_time_slice_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_time_slice_change.gcc.rsp" || exit 1
echo txe_thread_wait_abort.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_thread_wait_abort.gcc.rsp" || exit 1
echo txe_timer_activate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_timer_activate.gcc.rsp" || exit 1
echo txe_timer_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_timer_change.gcc.rsp" || exit 1
echo txe_timer_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_timer_create.gcc.rsp" || exit 1
echo txe_timer_deactivate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_timer_deactivate.gcc.rsp" || exit 1
echo txe_timer_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_timer_delete.gcc.rsp" || exit 1
echo txe_timer_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/txe_timer_info_get.gcc.rsp" || exit 1
echo tx_block_allocate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_allocate.gcc.rsp" || exit 1
echo tx_block_pool_cleanup.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_cleanup.gcc.rsp" || exit 1
echo tx_block_pool_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_create.gcc.rsp" || exit 1
echo tx_block_pool_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_delete.gcc.rsp" || exit 1
echo tx_block_pool_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_info_get.gcc.rsp" || exit 1
echo tx_block_pool_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_initialize.gcc.rsp" || exit 1
echo tx_block_pool_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_performance_info_get.gcc.rsp" || exit 1
echo tx_block_pool_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_performance_system_info_get.gcc.rsp" || exit 1
echo tx_block_pool_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_pool_prioritize.gcc.rsp" || exit 1
echo tx_block_release.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_block_release.gcc.rsp" || exit 1
echo tx_byte_allocate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_allocate.gcc.rsp" || exit 1
echo tx_byte_pool_cleanup.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_cleanup.gcc.rsp" || exit 1
echo tx_byte_pool_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_create.gcc.rsp" || exit 1
echo tx_byte_pool_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_delete.gcc.rsp" || exit 1
echo tx_byte_pool_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_info_get.gcc.rsp" || exit 1
echo tx_byte_pool_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_initialize.gcc.rsp" || exit 1
echo tx_byte_pool_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_performance_info_get.gcc.rsp" || exit 1
echo tx_byte_pool_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_performance_system_info_get.gcc.rsp" || exit 1
echo tx_byte_pool_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_prioritize.gcc.rsp" || exit 1
echo tx_byte_pool_search.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_pool_search.gcc.rsp" || exit 1
echo tx_byte_release.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_byte_release.gcc.rsp" || exit 1
echo tx_event_flags_cleanup.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_cleanup.gcc.rsp" || exit 1
echo tx_event_flags_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_create.gcc.rsp" || exit 1
echo tx_event_flags_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_delete.gcc.rsp" || exit 1
echo tx_event_flags_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_get.gcc.rsp" || exit 1
echo tx_event_flags_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_info_get.gcc.rsp" || exit 1
echo tx_event_flags_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_initialize.gcc.rsp" || exit 1
echo tx_event_flags_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_performance_info_get.gcc.rsp" || exit 1
echo tx_event_flags_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_performance_system_info_get.gcc.rsp" || exit 1
echo tx_event_flags_set.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_set.gcc.rsp" || exit 1
echo tx_event_flags_set_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_event_flags_set_notify.gcc.rsp" || exit 1
echo tx_initialize_high_level.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_initialize_high_level.gcc.rsp" || exit 1
echo tx_initialize_kernel_enter.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_initialize_kernel_enter.gcc.rsp" || exit 1
echo tx_initialize_kernel_setup.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_initialize_kernel_setup.gcc.rsp" || exit 1
echo tx_misra.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_misra.gcc.rsp" || exit 1
echo tx_mutex_cleanup.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_cleanup.gcc.rsp" || exit 1
echo tx_mutex_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_create.gcc.rsp" || exit 1
echo tx_mutex_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_delete.gcc.rsp" || exit 1
echo tx_mutex_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_get.gcc.rsp" || exit 1
echo tx_mutex_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_info_get.gcc.rsp" || exit 1
echo tx_mutex_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_initialize.gcc.rsp" || exit 1
echo tx_mutex_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_performance_info_get.gcc.rsp" || exit 1
echo tx_mutex_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_performance_system_info_get.gcc.rsp" || exit 1
echo tx_mutex_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_prioritize.gcc.rsp" || exit 1
echo tx_mutex_priority_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_priority_change.gcc.rsp" || exit 1
echo tx_mutex_put.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_mutex_put.gcc.rsp" || exit 1
echo tx_queue_cleanup.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_cleanup.gcc.rsp" || exit 1
echo tx_queue_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_create.gcc.rsp" || exit 1
echo tx_queue_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_delete.gcc.rsp" || exit 1
echo tx_queue_flush.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_flush.gcc.rsp" || exit 1
echo tx_queue_front_send.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_front_send.gcc.rsp" || exit 1
echo tx_queue_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_info_get.gcc.rsp" || exit 1
echo tx_queue_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_initialize.gcc.rsp" || exit 1
echo tx_queue_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_performance_info_get.gcc.rsp" || exit 1
echo tx_queue_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_performance_system_info_get.gcc.rsp" || exit 1
echo tx_queue_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_prioritize.gcc.rsp" || exit 1
echo tx_queue_receive.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_receive.gcc.rsp" || exit 1
echo tx_queue_send.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_send.gcc.rsp" || exit 1
echo tx_queue_send_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_queue_send_notify.gcc.rsp" || exit 1
echo tx_semaphore_ceiling_put.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_ceiling_put.gcc.rsp" || exit 1
echo tx_semaphore_cleanup.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_cleanup.gcc.rsp" || exit 1
echo tx_semaphore_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_create.gcc.rsp" || exit 1
echo tx_semaphore_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_delete.gcc.rsp" || exit 1
echo tx_semaphore_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_get.gcc.rsp" || exit 1
echo tx_semaphore_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_info_get.gcc.rsp" || exit 1
echo tx_semaphore_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_initialize.gcc.rsp" || exit 1
echo tx_semaphore_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_performance_info_get.gcc.rsp" || exit 1
echo tx_semaphore_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_performance_system_info_get.gcc.rsp" || exit 1
echo tx_semaphore_prioritize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_prioritize.gcc.rsp" || exit 1
echo tx_semaphore_put.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_put.gcc.rsp" || exit 1
echo tx_semaphore_put_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_semaphore_put_notify.gcc.rsp" || exit 1
echo tx_thread_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_create.gcc.rsp" || exit 1
echo tx_thread_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_delete.gcc.rsp" || exit 1
echo tx_thread_entry_exit_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_entry_exit_notify.gcc.rsp" || exit 1
echo tx_thread_identify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_identify.gcc.rsp" || exit 1
echo tx_thread_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_info_get.gcc.rsp" || exit 1
echo tx_thread_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_initialize.gcc.rsp" || exit 1
echo tx_thread_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_performance_info_get.gcc.rsp" || exit 1
echo tx_thread_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_performance_system_info_get.gcc.rsp" || exit 1
echo tx_thread_preemption_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_preemption_change.gcc.rsp" || exit 1
echo tx_thread_priority_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_priority_change.gcc.rsp" || exit 1
echo tx_thread_relinquish.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_relinquish.gcc.rsp" || exit 1
echo tx_thread_reset.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_reset.gcc.rsp" || exit 1
echo tx_thread_resume.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_resume.gcc.rsp" || exit 1
echo tx_thread_shell_entry.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_shell_entry.gcc.rsp" || exit 1
echo tx_thread_sleep.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_sleep.gcc.rsp" || exit 1
echo tx_thread_stack_analyze.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_stack_analyze.gcc.rsp" || exit 1
echo tx_thread_stack_error_handler.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_stack_error_handler.gcc.rsp" || exit 1
echo tx_thread_stack_error_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_stack_error_notify.gcc.rsp" || exit 1
echo tx_thread_suspend.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_suspend.gcc.rsp" || exit 1
echo tx_thread_system_preempt_check.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_system_preempt_check.gcc.rsp" || exit 1
echo tx_thread_system_resume.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_system_resume.gcc.rsp" || exit 1
echo tx_thread_system_suspend.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_system_suspend.gcc.rsp" || exit 1
echo tx_thread_terminate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_terminate.gcc.rsp" || exit 1
echo tx_thread_timeout.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_timeout.gcc.rsp" || exit 1
echo tx_thread_time_slice.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_time_slice.gcc.rsp" || exit 1
echo tx_thread_time_slice_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_time_slice_change.gcc.rsp" || exit 1
echo tx_thread_wait_abort.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_thread_wait_abort.gcc.rsp" || exit 1
echo tx_timer_activate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_activate.gcc.rsp" || exit 1
echo tx_timer_change.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_change.gcc.rsp" || exit 1
echo tx_timer_create.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_create.gcc.rsp" || exit 1
echo tx_timer_deactivate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_deactivate.gcc.rsp" || exit 1
echo tx_timer_delete.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_delete.gcc.rsp" || exit 1
echo tx_timer_expiration_process.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_expiration_process.gcc.rsp" || exit 1
echo tx_timer_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_info_get.gcc.rsp" || exit 1
echo tx_timer_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_initialize.gcc.rsp" || exit 1
echo tx_timer_performance_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_performance_info_get.gcc.rsp" || exit 1
echo tx_timer_performance_system_info_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_performance_system_info_get.gcc.rsp" || exit 1
echo tx_timer_system_activate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_system_activate.gcc.rsp" || exit 1
echo tx_timer_system_deactivate.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_system_deactivate.gcc.rsp" || exit 1
echo tx_timer_thread_entry.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_timer_thread_entry.gcc.rsp" || exit 1
echo tx_time_get.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_time_get.gcc.rsp" || exit 1
echo tx_time_set.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_time_set.gcc.rsp" || exit 1
echo tx_trace_buffer_full_notify.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_buffer_full_notify.gcc.rsp" || exit 1
echo tx_trace_disable.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_disable.gcc.rsp" || exit 1
echo tx_trace_enable.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_enable.gcc.rsp" || exit 1
echo tx_trace_event_filter.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_event_filter.gcc.rsp" || exit 1
echo tx_trace_event_unfilter.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_event_unfilter.gcc.rsp" || exit 1
echo tx_trace_initialize.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_initialize.gcc.rsp" || exit 1
echo tx_trace_interrupt_control.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_interrupt_control.gcc.rsp" || exit 1
echo tx_trace_isr_enter_insert.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_isr_enter_insert.gcc.rsp" || exit 1
echo tx_trace_isr_exit_insert.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_isr_exit_insert.gcc.rsp" || exit 1
echo tx_trace_object_register.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_object_register.gcc.rsp" || exit 1
echo tx_trace_object_unregister.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_object_unregister.gcc.rsp" || exit 1
echo tx_trace_user_event_insert.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/azure-rtos/threadx/common/src/tx_trace_user_event_insert.gcc.rsp" || exit 1
echo Cache.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Cache.gcc.rsp" || exit 1
echo Checks.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Checks.gcc.rsp" || exit 1
echo CLR_RT_DblLinkedList.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_DblLinkedList.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Array.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Array.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_ArrayList.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_ArrayList.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_BinaryBlob.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_BinaryBlob.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Delegate.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Delegate.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Delegate_List.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Delegate_List.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Finalizer.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Finalizer.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Lock.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Lock.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_LockRequest.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_LockRequest.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Node.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Node.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Queue.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Queue.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Stack.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Stack.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_String.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_String.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_Timer.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_Timer.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_WaitForObject.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapBlock_WaitForObject.gcc.rsp" || exit 1
echo CLR_RT_HeapCluster.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_HeapCluster.gcc.rsp" || exit 1
echo CLR_RT_Interop.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_Interop.gcc.rsp" || exit 1
echo CLR_RT_Memory.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_Memory.gcc.rsp" || exit 1
echo CLR_RT_ObjectToEvent_Destination.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_ObjectToEvent_Destination.gcc.rsp" || exit 1
echo CLR_RT_ObjectToEvent_Source.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_ObjectToEvent_Source.gcc.rsp" || exit 1
echo CLR_RT_RuntimeMemory.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_RuntimeMemory.gcc.rsp" || exit 1
echo CLR_RT_StackFrame.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_StackFrame.gcc.rsp" || exit 1
echo CLR_RT_SystemAssembliesTable.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_SystemAssembliesTable.gcc.rsp" || exit 1
echo CLR_RT_UnicodeHelper.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/CLR_RT_UnicodeHelper.gcc.rsp" || exit 1
echo Core.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Core.gcc.rsp" || exit 1
echo Execution.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Execution.gcc.rsp" || exit 1
echo GarbageCollector.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/GarbageCollector.gcc.rsp" || exit 1
echo GarbageCollector_Compaction.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/GarbageCollector_Compaction.gcc.rsp" || exit 1
echo GarbageCollector_ComputeReachabilityGraph.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/GarbageCollector_ComputeReachabilityGraph.gcc.rsp" || exit 1
echo GarbageCollector_Info.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/GarbageCollector_Info.gcc.rsp" || exit 1
echo Hardware.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Hardware/Hardware.gcc.rsp" || exit 1
echo Hardware_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Hardware/Hardware_stub.gcc.rsp" || exit 1
echo Interpreter.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Interpreter.gcc.rsp" || exit 1
echo InterruptHandler.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/InterruptHandler/InterruptHandler.gcc.rsp" || exit 1
echo InterruptHandler_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/InterruptHandler/InterruptHandler_stub.gcc.rsp" || exit 1
echo nanoSupport_CRC32.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/nanoSupport_CRC32.gcc.rsp" || exit 1
echo NativeEventDispatcher.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/NativeEventDispatcher/NativeEventDispatcher.gcc.rsp" || exit 1
echo NativeEventDispatcher_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/NativeEventDispatcher/NativeEventDispatcher_stub.gcc.rsp" || exit 1
echo Random.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Random.gcc.rsp" || exit 1
echo CLR_RT_HeapBlock_EndPoint.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/RPC/CLR_RT_HeapBlock_EndPoint.gcc.rsp" || exit 1
echo RPC_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/RPC/RPC_stub.gcc.rsp" || exit 1
echo BinaryFormatter.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Serialization/BinaryFormatter.gcc.rsp" || exit 1
echo BinaryFormatter_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Serialization/BinaryFormatter_stub.gcc.rsp" || exit 1
echo Streams.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Streams.gcc.rsp" || exit 1
echo StringTable.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/StringTable.gcc.rsp" || exit 1
echo StringTableData.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/StringTableData.gcc.rsp" || exit 1
echo Thread.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Thread.gcc.rsp" || exit 1
echo TypeSystem.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/TypeSystem.gcc.rsp" || exit 1
echo TypeSystemLookup.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/TypeSystemLookup.gcc.rsp" || exit 1
echo Various.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Core/Various.gcc.rsp" || exit 1
echo corlib_native.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native.gcc.rsp" || exit 1
echo corlib_native_System_AppDomain.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_AppDomain.gcc.rsp" || exit 1
echo corlib_native_System_Array.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Array.gcc.rsp" || exit 1
echo corlib_native_System_Attribute.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Attribute.gcc.rsp" || exit 1
echo corlib_native_System_BitConverter.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_BitConverter.gcc.rsp" || exit 1
echo corlib_native_System_Collections_ArrayList.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Collections_ArrayList.gcc.rsp" || exit 1
echo corlib_native_System_Convert.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Convert.gcc.rsp" || exit 1
echo corlib_native_System_DateTime.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_DateTime.gcc.rsp" || exit 1
echo corlib_native_System_Delegate.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Delegate.gcc.rsp" || exit 1
echo corlib_native_System_Diagnostics_Debug.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Diagnostics_Debug.gcc.rsp" || exit 1
echo corlib_native_System_Diagnostics_Debugger.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Diagnostics_Debugger.gcc.rsp" || exit 1
echo corlib_native_System_Double.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Double.gcc.rsp" || exit 1
echo corlib_native_System_Enum.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Enum.gcc.rsp" || exit 1
echo corlib_native_System_Exception.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Exception.gcc.rsp" || exit 1
echo corlib_native_System_GC.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_GC.gcc.rsp" || exit 1
echo corlib_native_System_Globalization_CultureInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Globalization_CultureInfo.gcc.rsp" || exit 1
echo corlib_native_System_Globalization_DateTimeFormat.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Globalization_DateTimeFormat.gcc.rsp" || exit 1
echo corlib_native_System_Guid.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Guid.gcc.rsp" || exit 1
echo corlib_native_System_MarshalByRefObject.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_MarshalByRefObject.gcc.rsp" || exit 1
echo corlib_native_System_MathInternal.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_MathInternal.gcc.rsp" || exit 1
echo corlib_native_System_MulticastDelegate.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_MulticastDelegate.gcc.rsp" || exit 1
echo corlib_native_System_Number.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Number.gcc.rsp" || exit 1
echo corlib_native_System_Object.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Object.gcc.rsp" || exit 1
echo corlib_native_System_Random.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Random.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_Assembly.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_Assembly.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_Binder.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_Binder.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_ConstructorInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_ConstructorInfo.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_FieldInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_FieldInfo.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_MemberInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_MemberInfo.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_MethodBase.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_MethodBase.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_PropertyInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_PropertyInfo.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_RuntimeFieldInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_RuntimeFieldInfo.gcc.rsp" || exit 1
echo corlib_native_System_Reflection_RuntimeMethodInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Reflection_RuntimeMethodInfo.gcc.rsp" || exit 1
echo corlib_native_System_RuntimeType.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_RuntimeType.gcc.rsp" || exit 1
echo corlib_native_System_Runtime_CompilerServices_RuntimeHelpers.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Runtime_CompilerServices_RuntimeHelpers.gcc.rsp" || exit 1
echo corlib_native_System_Runtime_Remoting_RemotingServices.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Runtime_Remoting_RemotingServices.gcc.rsp" || exit 1
echo corlib_native_System_String.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_String.gcc.rsp" || exit 1
echo corlib_native_System_Threading_AutoResetEvent.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_AutoResetEvent.gcc.rsp" || exit 1
echo corlib_native_System_Threading_Interlocked.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_Interlocked.gcc.rsp" || exit 1
echo corlib_native_System_Threading_ManualResetEvent.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_ManualResetEvent.gcc.rsp" || exit 1
echo corlib_native_System_Threading_Monitor.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_Monitor.gcc.rsp" || exit 1
echo corlib_native_System_Threading_SpinWait.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_SpinWait.gcc.rsp" || exit 1
echo corlib_native_System_Threading_Thread.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_Thread.gcc.rsp" || exit 1
echo corlib_native_System_Threading_Timer.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_Timer.gcc.rsp" || exit 1
echo corlib_native_System_Threading_WaitHandle.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Threading_WaitHandle.gcc.rsp" || exit 1
echo corlib_native_System_TimeSpan.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_TimeSpan.gcc.rsp" || exit 1
echo corlib_native_System_Type.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_Type.gcc.rsp" || exit 1
echo corlib_native_System_ValueType.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_ValueType.gcc.rsp" || exit 1
echo corlib_native_System_WeakReference.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/CorLib/corlib_native_System_WeakReference.gcc.rsp" || exit 1
echo Debugger.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Debugger/Debugger.gcc.rsp" || exit 1
echo Debugger_full.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Debugger/Debugger_full.gcc.rsp" || exit 1
echo Debugger_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Debugger/Debugger_stub.gcc.rsp" || exit 1
echo Diagnostics_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Diagnostics/Diagnostics_stub.gcc.rsp" || exit 1
echo Info.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Diagnostics/Info.gcc.rsp" || exit 1
echo Profile.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Diagnostics/Profile.gcc.rsp" || exit 1
echo Profiler.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Diagnostics/Profiler.gcc.rsp" || exit 1
echo base64.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Helpers/Base64/base64.gcc.rsp" || exit 1
echo nanoprintf.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Helpers/nanoprintf/nanoprintf.gcc.rsp" || exit 1
echo Messaging.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Messaging/Messaging.gcc.rsp" || exit 1
echo Messaging_stub.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Messaging/Messaging_stub.gcc.rsp" || exit 1
echo CLRStartup.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/Startup/CLRStartup.gcc.rsp" || exit 1
echo nf_native_system_math.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/System.Math/nf_native_system_math.gcc.rsp" || exit 1
echo nf_native_system_math_System_Math.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/System.Math/nf_native_system_math_System_Math.gcc.rsp" || exit 1
echo WireProtocol_App_Interface.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/WireProtocol/WireProtocol_App_Interface.gcc.rsp" || exit 1
echo WireProtocol_HAL_Interface.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/WireProtocol/WireProtocol_HAL_Interface.gcc.rsp" || exit 1
echo WireProtocol_Message.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/WireProtocol/WireProtocol_Message.gcc.rsp" || exit 1
echo WireProtocol_MonitorCommands.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/CLR/WireProtocol/WireProtocol_MonitorCommands.gcc.rsp" || exit 1
echo nanoHAL_Boot.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/HAL/nanoHAL_Boot.gcc.rsp" || exit 1
echo nanoHAL_Capabilites.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/HAL/nanoHAL_Capabilites.gcc.rsp" || exit 1
echo nanoHAL_ConfigurationManager.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/HAL/nanoHAL_ConfigurationManager.gcc.rsp" || exit 1
echo nanoHAL_SystemEvents.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/HAL/nanoHAL_SystemEvents.gcc.rsp" || exit 1
echo nanoHAL_SystemInformation.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/HAL/nanoHAL_SystemInformation.gcc.rsp" || exit 1
echo nanoHAL_Time.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/HAL/nanoHAL_Time.gcc.rsp" || exit 1
echo nanoHAL_Watchdog.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/HAL/nanoHAL_Watchdog.gcc.rsp" || exit 1
echo Graphics.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Graphics.gcc.rsp" || exit 1
echo GraphicsDriver.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/GraphicsDriver.gcc.rsp" || exit 1
echo GraphicsMemoryHeap.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/GraphicsMemoryHeap.gcc.rsp" || exit 1
echo Bitmap_Decoder.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Bmp/Bitmap_Decoder.gcc.rsp" || exit 1
echo Font.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Fonts/Font.gcc.rsp" || exit 1
echo font8x8.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Fonts/font8x8.gcc.rsp" || exit 1
echo Gif.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Gif/Gif.gcc.rsp" || exit 1
echo GifDecoder.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Gif/GifDecoder.gcc.rsp" || exit 1
echo lzwread.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Gif/lzwread.gcc.rsp" || exit 1
echo jbytearraydatasrc.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jbytearraydatasrc.gcc.rsp" || exit 1
echo jcapimin.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcapimin.gcc.rsp" || exit 1
echo jcapistd.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcapistd.gcc.rsp" || exit 1
echo jccoefct.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jccoefct.gcc.rsp" || exit 1
echo jccolor.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jccolor.gcc.rsp" || exit 1
echo jcdctmgr.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcdctmgr.gcc.rsp" || exit 1
echo jchuff.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jchuff.gcc.rsp" || exit 1
echo jcinit.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcinit.gcc.rsp" || exit 1
echo jcmainct.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcmainct.gcc.rsp" || exit 1
echo jcmarker.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcmarker.gcc.rsp" || exit 1
echo jcmaster.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcmaster.gcc.rsp" || exit 1
echo jcomapi.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcomapi.gcc.rsp" || exit 1
echo jcparam.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcparam.gcc.rsp" || exit 1
echo jcphuff.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcphuff.gcc.rsp" || exit 1
echo jcprepct.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcprepct.gcc.rsp" || exit 1
echo jcsample.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jcsample.gcc.rsp" || exit 1
echo jctrans.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jctrans.gcc.rsp" || exit 1
echo jdapimin.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdapimin.gcc.rsp" || exit 1
echo jdapistd.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdapistd.gcc.rsp" || exit 1
echo jdcoefct.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdcoefct.gcc.rsp" || exit 1
echo jdcolor.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdcolor.gcc.rsp" || exit 1
echo jddctmgr.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jddctmgr.gcc.rsp" || exit 1
echo jdhuff.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdhuff.gcc.rsp" || exit 1
echo jdinput.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdinput.gcc.rsp" || exit 1
echo jdmainct.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdmainct.gcc.rsp" || exit 1
echo jdmarker.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdmarker.gcc.rsp" || exit 1
echo jdmaster.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdmaster.gcc.rsp" || exit 1
echo jdmerge.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdmerge.gcc.rsp" || exit 1
echo jdphuff.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdphuff.gcc.rsp" || exit 1
echo jdpostct.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdpostct.gcc.rsp" || exit 1
echo jdsample.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdsample.gcc.rsp" || exit 1
echo jdtrans.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jdtrans.gcc.rsp" || exit 1
echo jerror.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jerror.gcc.rsp" || exit 1
echo jfdctflt.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jfdctflt.gcc.rsp" || exit 1
echo jfdctfst.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jfdctfst.gcc.rsp" || exit 1
echo jfdctint.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jfdctint.gcc.rsp" || exit 1
echo jidctflt.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jidctflt.gcc.rsp" || exit 1
echo jidctfst.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jidctfst.gcc.rsp" || exit 1
echo jidctint.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jidctint.gcc.rsp" || exit 1
echo jidctred.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jidctred.gcc.rsp" || exit 1
echo jmemmgr.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jmemmgr.gcc.rsp" || exit 1
echo jmemnanoclr.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jmemnanoclr.gcc.rsp" || exit 1
echo Jpeg.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/Jpeg.gcc.rsp" || exit 1
echo jquant1.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jquant1.gcc.rsp" || exit 1
echo jquant2.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jquant2.gcc.rsp" || exit 1
echo jutils.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/jutils.gcc.rsp" || exit 1
echo mcbcr.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/mcbcr.gcc.rsp" || exit 1
echo mfint.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/mfint.gcc.rsp" || exit 1
echo miint.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/miint.gcc.rsp" || exit 1
echo pfint.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/pfint.gcc.rsp" || exit 1
echo piint.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/piint.gcc.rsp" || exit 1
echo transupp.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Core/Support/Jpeg/transupp.gcc.rsp" || exit 1
echo ST7789_240x240.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Displays/ST7789_240x240.gcc.rsp" || exit 1
echo nanoFramework_Graphics.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Native/nanoFramework_Graphics.gcc.rsp" || exit 1
echo nanoFramework_Graphics_nanoFramework_UI_Bitmap.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Native/nanoFramework_Graphics_nanoFramework_UI_Bitmap.gcc.rsp" || exit 1
echo nanoFramework_Graphics_nanoFramework_UI_DisplayControl.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Native/nanoFramework_Graphics_nanoFramework_UI_DisplayControl.gcc.rsp" || exit 1
echo nanoFramework_Graphics_nanoFramework_UI_Font.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/Graphics/Native/nanoFramework_Graphics_nanoFramework_UI_Font.gcc.rsp" || exit 1
echo TouchPanel.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Graphics/TouchPanel/Core/TouchPanel.gcc.rsp" || exit 1
echo nf_system_resourcemanager.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.ResourceManager/nf_system_resourcemanager.gcc.rsp" || exit 1
echo nf_system_resourcemanager_nanoFramework_Runtime_Native_ResourceUtility.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.ResourceManager/nf_system_resourcemanager_nanoFramework_Runtime_Native_ResourceUtility.gcc.rsp" || exit 1
echo nf_system_resourcemanager_System_Resources_ResourceManager.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.ResourceManager/nf_system_resourcemanager_System_Resources_ResourceManager.gcc.rsp" || exit 1
echo nf_rt_events_native.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Events/nf_rt_events_native.gcc.rsp" || exit 1
echo nf_rt_events_native_nanoFramework_Runtime_Events_EventSink.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Events/nf_rt_events_native_nanoFramework_Runtime_Events_EventSink.gcc.rsp" || exit 1
echo nf_rt_events_native_nanoFramework_Runtime_Events_NativeEventDispatcher.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Events/nf_rt_events_native_nanoFramework_Runtime_Events_NativeEventDispatcher.gcc.rsp" || exit 1
echo nf_rt_events_native_nanoFramework_Runtime_Events_WeakDelegate.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Events/nf_rt_events_native_nanoFramework_Runtime_Events_WeakDelegate.gcc.rsp" || exit 1
echo nf_rt_native.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Native/nf_rt_native.gcc.rsp" || exit 1
echo nf_rt_native_nanoFramework_Runtime_Hardware_SystemInfo.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Native/nf_rt_native_nanoFramework_Runtime_Hardware_SystemInfo.gcc.rsp" || exit 1
echo nf_rt_native_nanoFramework_Runtime_Native_ExecutionConstraint.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Native/nf_rt_native_nanoFramework_Runtime_Native_ExecutionConstraint.gcc.rsp" || exit 1
echo nf_rt_native_nanoFramework_Runtime_Native_GC.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Native/nf_rt_native_nanoFramework_Runtime_Native_GC.gcc.rsp" || exit 1
echo nf_rt_native_nanoFramework_Runtime_Native_Power.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Native/nf_rt_native_nanoFramework_Runtime_Native_Power.gcc.rsp" || exit 1
echo nf_rt_native_nanoFramework_Runtime_Native_Rtc_stubs.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Native/nf_rt_native_nanoFramework_Runtime_Native_Rtc_stubs.gcc.rsp" || exit 1
echo nf_rt_native_System_Environment.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.Runtime.Native/nf_rt_native_System_Environment.gcc.rsp" || exit 1
echo nf_system_collections.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Collections/nf_system_collections.gcc.rsp" || exit 1
echo nf_system_collections_System_Collections_Hashtable.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Collections/nf_system_collections_System_Collections_Hashtable.gcc.rsp" || exit 1
echo nf_system_collections_System_Collections_Hashtable__HashtableEnumerator.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Collections/nf_system_collections_System_Collections_Hashtable__HashtableEnumerator.gcc.rsp" || exit 1
echo nf_system_collections_System_Collections_Queue.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Collections/nf_system_collections_System_Collections_Queue.gcc.rsp" || exit 1
echo nf_system_collections_System_Collections_Stack.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Collections/nf_system_collections_System_Collections_Stack.gcc.rsp" || exit 1
echo nf_system_text.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Text/nf_system_text.gcc.rsp" || exit 1
echo nf_system_text_System_Text_UTF8Decoder.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Text/nf_system_text_System_Text_UTF8Decoder.gcc.rsp" || exit 1
echo nf_system_text_System_Text_UTF8Encoding.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/nanoFramework.System.Text/nf_system_text_System_Text_UTF8Encoding.gcc.rsp" || exit 1
echo AsyncCompletions.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/AsyncProcCall/AsyncCompletions.gcc.rsp" || exit 1
echo AsyncContinuations.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/AsyncProcCall/AsyncContinuations.gcc.rsp" || exit 1
echo nanoPAL_BlockStorage.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/BlockStorage/nanoPAL_BlockStorage.gcc.rsp" || exit 1
echo COM_stubs.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/COM/COM_stubs.gcc.rsp" || exit 1
echo GenericPort_stubs.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/COM/GenericPort_stubs.gcc.rsp" || exit 1
echo nanoPAL_NativeDouble.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/Double/nanoPAL_NativeDouble.gcc.rsp" || exit 1
echo nanoPAL_Events.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/Events/nanoPAL_Events.gcc.rsp" || exit 1
echo nanoPAL_Events_driver.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/Events/nanoPAL_Events_driver.gcc.rsp" || exit 1
echo nanoPAL_Events_functions.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/Events/nanoPAL_Events_functions.gcc.rsp" || exit 1
echo nanoPAL_Network_stubs.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/nanoPAL_Network_stubs.gcc.rsp" || exit 1
echo nanoPAL_PerformanceCounters_stubs.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/PAL/Profiler/nanoPAL_PerformanceCounters_stubs.gcc.rsp" || exit 1
echo sys_dev_gpio_native.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/System.Device.Gpio/sys_dev_gpio_native.gcc.rsp" || exit 1
echo sys_dev_gpio_native_System_Device_Gpio_GpioController.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/System.Device.Gpio/sys_dev_gpio_native_System_Device_Gpio_GpioController.gcc.rsp" || exit 1
echo sys_dev_gpio_native_System_Device_Gpio_GpioPin.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/System.Device.Gpio/sys_dev_gpio_native_System_Device_Gpio_GpioPin.gcc.rsp" || exit 1
echo nanoHAL_Spi.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/nf-interpreter/src/System.Device.Spi/nanoHAL_Spi.gcc.rsp" || exit 1
echo critical_section.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_sync/critical_section.gcc.rsp" || exit 1
echo lock_core.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_sync/lock_core.gcc.rsp" || exit 1
echo mutex.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_sync/mutex.gcc.rsp" || exit 1
echo sem.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_sync/sem.gcc.rsp" || exit 1
echo time.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_time/time.gcc.rsp" || exit 1
echo timeout_helper.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_time/timeout_helper.gcc.rsp" || exit 1
echo datetime.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_util/datetime.gcc.rsp" || exit 1
echo pheap.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_util/pheap.gcc.rsp" || exit 1
echo queue.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/common/pico_util/queue.gcc.rsp" || exit 1
echo claim.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_claim/claim.gcc.rsp" || exit 1
echo clocks.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_clocks/clocks.gcc.rsp" || exit 1
echo gpio.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_gpio/gpio.gcc.rsp" || exit 1
echo irq.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_irq/irq.gcc.rsp" || exit 1
echo pll.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_pll/pll.gcc.rsp" || exit 1
echo sync.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_sync/sync.gcc.rsp" || exit 1
echo timer.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_timer/timer.gcc.rsp" || exit 1
echo uart.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_uart/uart.gcc.rsp" || exit 1
echo vreg.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_vreg/vreg.gcc.rsp" || exit 1
echo watchdog.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.gcc.rsp" || exit 1
echo xosc.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_xosc/xosc.gcc.rsp" || exit 1
echo bootrom.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_bootrom/bootrom.gcc.rsp" || exit 1
echo double_init_rom.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_double/double_init_rom.gcc.rsp" || exit 1
echo double_math.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_double/double_math.gcc.rsp" || exit 1
echo float_init_rom.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_float/float_init_rom.gcc.rsp" || exit 1
echo float_math.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_float/float_math.gcc.rsp" || exit 1
echo pico_malloc.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.gcc.rsp" || exit 1
echo platform.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_platform/platform.gcc.rsp" || exit 1
echo printf.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_printf/printf.gcc.rsp" || exit 1
echo runtime.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_runtime/runtime.gcc.rsp" || exit 1
echo binary_info.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_standard_link/binary_info.gcc.rsp" || exit 1
echo new_delete.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_standard_link/new_delete.gcc.rsp" || exit 1
echo stdio.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_stdio/stdio.gcc.rsp" || exit 1
echo stdio_uart.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_stdio_uart/stdio_uart.gcc.rsp" || exit 1
echo stdlib.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_3_/ExternalRepositories/pico-sdk/src/rp2_common/pico_stdlib/stdlib.gcc.rsp" || exit 1
echo nanoCRT.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_1_/Common/nanoCRT.gcc.rsp" || exit 1
echo nanoHAL.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_1_/Common/nanoHAL.gcc.rsp" || exit 1
echo platform_heap.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_1_/Common/platform_heap.gcc.rsp" || exit 1
echo targetHAL_Time.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_1_/Common/targetHAL_Time.gcc.rsp" || exit 1
echo targetPAL_Events.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_1_/Common/targetPAL_Events.gcc.rsp" || exit 1
echo targetPAL_Time.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_1_/Common/targetPAL_Time.gcc.rsp" || exit 1
echo WireProtocol_HAL_Interface.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_1_/Common/WireProtocol/WireProtocol_HAL_Interface.gcc.rsp" || exit 1
echo WireProtocol_ReceiverThread.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_1_/Common/WireProtocol/WireProtocol_ReceiverThread.gcc.rsp" || exit 1
echo Board_PICO.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/AdaptionLayer/Board_PICO.gcc.rsp" || exit 1
echo targetHAL.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/AdaptionLayer/targetHAL.gcc.rsp" || exit 1
echo targetHAL_ConfigurationManager.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/AdaptionLayer/targetHAL_ConfigurationManager.gcc.rsp" || exit 1
echo targetHAL_Power.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/AdaptionLayer/targetHAL_Power.gcc.rsp" || exit 1
echo targetPAL.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/AdaptionLayer/targetPAL.gcc.rsp" || exit 1
echo target_common.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/AdaptionLayer/target_common.gcc.rsp" || exit 1
echo wpUSART_Communications.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/AdaptionLayer/wpUSART_Communications.gcc.rsp" || exit 1
echo Audio.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Features/Audio.gcc.rsp" || exit 1
echo Ethernet.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Features/Ethernet.gcc.rsp" || exit 1
echo FDCAN.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Features/FDCAN.gcc.rsp" || exit 1
echo Graphics_Memory.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/Features/Graphics/Graphics_Memory.gcc.rsp" || exit 1
echo Initialize_Graphics.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/Features/Graphics/Initialize_Graphics.gcc.rsp" || exit 1
echo Spi_To_Display.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/Features/Graphics/Spi_To_Display.gcc.rsp" || exit 1
echo GPIO.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/Features/IO/GPIO.gcc.rsp" || exit 1
echo MicroSD.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Features/MicroSD.gcc.rsp" || exit 1
echo RealTimeClock.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Features/RealTimeClock.gcc.rsp" || exit 1
echo USB.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Features/USB.gcc.rsp" || exit 1
echo main.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/main.gcc.rsp" || exit 1
echo Device_BlockStorage.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Memory/Device_BlockStorage.gcc.rsp" || exit 1
echo Memory.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/Memory/Memory.gcc.rsp" || exit 1
echo platform_BlockStorage.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Memory/platform_BlockStorage.gcc.rsp" || exit 1
echo target_BlockStorage.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Memory/target_BlockStorage.gcc.rsp" || exit 1
echo Target_BlockStorage_STM32FlashDriver.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Memory/Target_BlockStorage_STM32FlashDriver.gcc.rsp" || exit 1
echo Clock_configuration.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Startup/Clock_configuration.gcc.rsp" || exit 1
echo CLR_RT_InteropAssembliesTable.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/Startup/CLR_RT_InteropAssembliesTable.gcc.rsp" || exit 1
echo CLR_Startup_Thread.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Startup/CLR_Startup_Thread.gcc.rsp" || exit 1
echo nanoFramework_Threads.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Startup/nanoFramework_Threads.gcc.rsp" || exit 1
echo system_stm32h7xx.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Startup/system_stm32h7xx.gcc.rsp" || exit 1
