#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0)
#include "esp_vfs_semihost.h"

#include "esp_log.h"
const static char *TAG = "semihost_test";

#define ESP_ENOTSUP_WIN 129
#define ESP_ENOTSUP_UNIX 95

#define SYSCALL_INSTR           "break 1,1\n"
#define SYS_OPEN                0x01
#define SYS_CLOSE               0x02
#define SYS_WRITE               0x05
#define SYS_READ                0x06
#define SYS_SEEK                0x0A
#define SYS_DRVINFO             0xE0
#define O_BINARY                0

typedef struct {
    int ver;
} drv_info_t;

static inline void done(void)
{
    ESP_LOGI(TAG, "CPU[%d]: [ DONE ]", xPortGetCoreID());
    while (1) {
        vTaskDelay(1);
    }
}

static inline int generic_syscall(int sys_nr, int arg1, int arg2, int arg3, int arg4,
                                  int *ret_errno)
{
    int host_ret, host_errno;
    int core_id = xPortGetCoreID();
    if (!esp_cpu_in_ocd_debug_mode()) {
        *ret_errno = EIO;
        return -1;
    }
    ESP_LOGI(TAG, "CPU[%d]: -> syscall 0x%x, args: 0x%x, 0x%x, 0x%x, 0x%x", core_id, sys_nr, arg1, arg2, arg3, arg4);
    __asm__ volatile (
        "mov a2, %[sys_nr]\n" \
        "mov a3, %[arg1]\n" \
        "mov a4, %[arg2]\n" \
        "mov a5, %[arg3]\n" \
        "mov a6, %[arg4]\n" \
        SYSCALL_INSTR \
        "mov %[host_ret], a2\n" \
        "mov %[host_errno], a3\n" \
        :[host_ret] "=r" (host_ret),[host_errno] "=r" (host_errno)
        :[sys_nr] "r" (sys_nr),[arg1] "r" (arg1),[arg2] "r" (arg2),[arg3] "r" (arg3),
        [arg4] "r" (arg4)
        : "a2","a3","a4","a5","a6");
    *ret_errno = host_errno;
    ESP_LOGI(TAG, "CPU[%d]: <- return:%d, errno:%d", core_id, host_ret, host_errno);
    return host_ret;
}

static inline int semihosting_wrong_args(int wrong_arg)
{
    int core_id = xPortGetCoreID();
    ESP_LOGI(TAG, "Started wrong args test for a core #%d. Arg is: %x", core_id, wrong_arg);
    int test_errno, syscall_ret;
    char fname[32];
    snprintf(fname, sizeof(fname) - 1, "/test_read.%d", core_id);

    ESP_LOGI(TAG, "CPU[%d]:------ wrong SYSCALL -------", core_id);
    syscall_ret = generic_syscall(wrong_arg, 0, 0, 0, 0, &test_errno);
    assert(syscall_ret == -1);
    assert((test_errno == ESP_ENOTSUP_WIN) || (test_errno == ESP_ENOTSUP_UNIX));

    /**** SYS_DRVINFO ****/
    ESP_LOGI(TAG, "CPU[%d]:------ SYS_DRVINFO test -------", core_id);
    int flags = 0;
    drv_info_t drv_info = {
        .ver = 1
    };

    ESP_LOGI(TAG, "CPU[%d]: wrong address", core_id);
    syscall_ret = generic_syscall(SYS_DRVINFO, wrong_arg, sizeof(drv_info), flags, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EINVAL);

    ESP_LOGI(TAG, "CPU[%d]: wrong size", core_id);
    syscall_ret = generic_syscall(SYS_DRVINFO, (int)&drv_info, wrong_arg, flags, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EINVAL);

    /**** SYS_OPEN ****/
    ESP_LOGI(TAG, "CPU[%d]:------ SYS_OPEN test -------", core_id);

    ESP_LOGI(TAG, "CPU[%d]: wrong fname_addr", core_id);
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,2,0,0)
    syscall_ret = generic_syscall(SYS_OPEN, wrong_arg, O_RDWR | O_BINARY, strlen(fname) + 1, 0, &test_errno);
#else
    syscall_ret = generic_syscall(SYS_OPEN, wrong_arg, strlen(fname) + 1, O_RDWR | O_BINARY, 0, &test_errno);
#endif
    assert(syscall_ret == -1);
    assert(test_errno == ENOMEM);

    ESP_LOGI(TAG, "CPU[%d]: wrong fname_len", core_id);
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,2,0,0)
    syscall_ret = generic_syscall(SYS_OPEN, (int)fname, O_RDWR | O_BINARY, wrong_arg, 0, &test_errno);
#else
    syscall_ret = generic_syscall(SYS_OPEN, (int)fname, wrong_arg, O_RDWR | O_BINARY, 0, &test_errno);
#endif
    assert(syscall_ret == -1);
    assert(test_errno == ENOMEM);

    ESP_LOGI(TAG, "CPU[%d]: wrong flags", core_id);
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,2,0,0)
    syscall_ret = generic_syscall(SYS_OPEN, (int)fname, wrong_arg, strlen(fname) + 1, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EINVAL);
#else
    syscall_ret = generic_syscall(SYS_OPEN, (int)fname, strlen(fname) + 1, wrong_arg, 0, &test_errno);
    assert(syscall_ret == -1);
    /* For the semihosting v0 we have no a flags checking so the returning error is solely depends on the platform's
    open-syscall implementation which is different from platform to platform */
#endif

    /**** Open the file correctly ****/
    /* Note: we are not using here `open` because VFS has system of global and local file descriptors, and open will
    return the global one (inside the target scope). For interacting via `generic_syscall` we need the local one -
    a file descriptor assigned to the file by the host */
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,2,0,0)
    int fd = generic_syscall(SYS_OPEN, (int)fname, O_RDWR | O_BINARY, strlen(fname) + 1, 0, &test_errno);
#else
    int fd = generic_syscall(SYS_OPEN, (int)fname, strlen(fname) + 1, O_RDWR | O_BINARY, 0, &test_errno);
#endif
    if (fd == -1) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to open file `%s` (%d)!", core_id, fname, errno);
        assert(false);
    }
    ESP_LOGI(TAG, "CPU[%d]: Opened %s, fd: %d ", core_id, fname, fd);

    /**** SYS_WRITE ****/
    ESP_LOGI(TAG, "CPU[%d]:------ SYS_WRITE test -------", core_id);
    char *data = "test";
    int write_size = strlen(data) + 1;

    ESP_LOGI(TAG, "CPU[%d]: wrong fd", core_id);
    syscall_ret = generic_syscall(SYS_WRITE, wrong_arg, (int)data, write_size, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EBADF);

    test_errno = 0;
    ESP_LOGI(TAG, "CPU[%d]: wrong data pointer", core_id);
    syscall_ret = generic_syscall(SYS_WRITE, fd, wrong_arg, write_size, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EIO);

    ESP_LOGI(TAG, "CPU[%d]: wrong data size", core_id);
    syscall_ret = generic_syscall(SYS_WRITE, fd, (int)data, wrong_arg, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == ENOMEM);

    /**** SYS_READ ****/
    ESP_LOGI(TAG, "CPU[%d]:------ SYS_READ test -------", core_id);
    char read_data[4] = {0};
    int read_size = sizeof(read_data);

    ESP_LOGI(TAG, "CPU[%d]: wrong fd", core_id);
    syscall_ret = generic_syscall(SYS_READ, wrong_arg, (int)read_data, read_size, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EBADF);

    ESP_LOGI(TAG, "CPU[%d]: wrong data pointer", core_id);
    syscall_ret = generic_syscall(SYS_READ, fd, wrong_arg, read_size, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EIO);

    ESP_LOGI(TAG, "CPU[%d]: wrong data size", core_id);
    syscall_ret = generic_syscall(SYS_READ, fd, (int)read_data, wrong_arg, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == ENOMEM);

    /**** SYS_SEEK ****/
    ESP_LOGI(TAG, "CPU[%d]:------ SYS_SEEK test -------", core_id);
    int seek_offset = 3;

    ESP_LOGI(TAG, "CPU[%d]: wrong fd", core_id);
    syscall_ret = generic_syscall(SYS_SEEK, wrong_arg, seek_offset, SEEK_SET, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EBADF);

    ESP_LOGI(TAG, "CPU[%d]: wrong offset", core_id);
    syscall_ret = generic_syscall(SYS_SEEK, fd, wrong_arg, SEEK_SET, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EINVAL);

    ESP_LOGI(TAG, "CPU[%d]: wrong mode", core_id);
    syscall_ret = generic_syscall(SYS_SEEK, fd, seek_offset, wrong_arg, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EINVAL);

    /**** SYS_CLOSE ****/
    ESP_LOGI(TAG, "CPU[%d]:------ SYS_CLOSE test -------", core_id);

    ESP_LOGI(TAG, "CPU[%d]: wrong fd", core_id);
    syscall_ret = generic_syscall(SYS_CLOSE, wrong_arg, 0, 0, 0, &test_errno);
    assert(syscall_ret == -1);
    assert(test_errno == EBADF);

    /**** Close the file correctly ****/
    /* Note: we are not using here `close` because VFS has system of global and local file descriptors, and open will
    return the global one (inside the target scope). For interacting via `generic_syscall` we need the local one -
    a file descriptor assigned to the file by the host */
    ESP_LOGI(TAG, "Closing the files");
    syscall_ret = generic_syscall(SYS_CLOSE, fd, 0, 0, 0, &test_errno);
    if (syscall_ret == -1) {
        ESP_LOGE(TAG, "CPU[%d] :Failed to close input file (%d)!", core_id, errno);
    }

    return 0;
}

static void semihost_task(void *pvParameter)
{
    uint8_t s_buf[512];
    int core_id = xPortGetCoreID();
    char fname[32];
    esp_err_t ret;
    ESP_LOGI(TAG, "Started test thread for a core #%d", core_id);

    /**** Init ****/
    ESP_LOGI(TAG, "CPU[%d]: Initialization", core_id);
    if (core_id == 0) {
        ret = esp_vfs_semihost_register("/host", NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to register semihost driver (%s)!", core_id, esp_err_to_name(ret));
            return;
        }
#if !CONFIG_FREERTOS_UNICORE
        xTaskCreatePinnedToCore(&semihost_task, "semihost_task1", 4096, xTaskGetCurrentTaskHandle(), 5, NULL, 1);
        vTaskDelay(1);
#endif
    }

    /**** Opening files ****/
    snprintf(fname, sizeof(fname)-1, "/host/test_write.%d", core_id);
    ESP_LOGI(TAG, "CPU[%d]: Opening %s", core_id, fname);
    FILE *f_out = fopen(fname, "w+");
    if (f_out == NULL) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to open file for writing (%d)!", core_id, errno);
        assert(false);
    }
    snprintf(fname, sizeof(fname) - 1, "/host/test_read.%d", core_id);
    ESP_LOGI(TAG, "CPU[%d]: Opening %s", core_id, fname);
    int fd_in = open(fname, O_RDONLY, 0);
    if (fd_in == -1) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to open file for reading (%d)!", core_id, errno);
        fclose(f_out);
        assert(false);
    }

    /**** Seeking test ****/
    int cursor = lseek(fd_in, 11, SEEK_SET);
    int cursor_expect = 11;
    ESP_LOGI(TAG, "CPU[%d]: SEEK_SET to %d", core_id, cursor);
    if (cursor != cursor_expect){
        ESP_LOGE(TAG, "CPU[%d]: Wrong cursor location (%d != %d)!", core_id, cursor, cursor_expect);
        assert(false);
    }

    cursor = lseek(fd_in, 11, SEEK_CUR);
    cursor_expect = 22;
    ESP_LOGI(TAG, "CPU[%d]: SEEK_CUR to %d", core_id, cursor);
    if (cursor != cursor_expect){
        ESP_LOGE(TAG, "CPU[%d]: Wrong cursor location (%d != %d)!", core_id, cursor, cursor_expect);
        assert(false);
    }

    cursor = lseek(fd_in, -1, SEEK_END);
    cursor_expect = lseek(fd_in, 0, SEEK_END) - 1;
    ESP_LOGI(TAG, "CPU[%d]: SEEK_END to %d", core_id, cursor);
    if (cursor != cursor_expect){
        ESP_LOGE(TAG, "CPU[%d]: Wrong cursor location (%d != %d)!", core_id, cursor, cursor_expect);
        assert(false);
    }

    cursor = lseek(fd_in, 0, SEEK_SET);
    ESP_LOGI(TAG, "CPU[%d]: Set to the beginning (cur: %d)", core_id, cursor);


    /**** Copy: In->Buf->Out ****/
    ESP_LOGI(TAG, "CPU[%d]: Writing test_read.%d ->  test_write.%d", core_id, core_id, core_id);
    ssize_t read_bytes;
    int count = 0;
    do {
        read_bytes = read(fd_in, s_buf, sizeof(s_buf)); // in -> buffer
        if (read_bytes == -1) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to read file (%d)!", core_id, errno);
        } else if (read_bytes > 0) {
            fwrite(s_buf, 1, read_bytes, f_out); // buffer -> out
            count += read_bytes;
        }
    } while (read_bytes > 0);

    /***** Checking *****/
    long int f_size = ftell(f_out);
    if (count != f_size) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to determine file size! (size is %ld)", core_id, f_size);
        assert(false);
    }

    ESP_LOGI(TAG, "CPU[%d]: Read %d bytes", core_id, count);
    ESP_LOGI(TAG, "CPU[%d]: Wrote %ld bytes", core_id, f_size);

    /***** De-init *****/
    ESP_LOGI(TAG, "Closing the files");
    if (close(fd_in) == -1) {
        ESP_LOGE(TAG, "CPU[%d] :Failed to close input file (%d)!", core_id, errno);
    }
    if (fclose(f_out) != 0) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to close output file (%d)!", core_id, errno);
    }
    ESP_LOGI(TAG, "CPU[%d]: Closed files", core_id);

    if (core_id == 0) {
#if !CONFIG_FREERTOS_UNICORE
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
#endif
        ESP_LOGI(TAG, "CPU[%d]: Unregister host FS", core_id);
        ret = esp_vfs_semihost_unregister("/host");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to unregister semihost driver (%s)!", core_id, esp_err_to_name(ret));
            assert(false);
        }
    } else {
        xTaskNotifyGive((TaskHandle_t)pvParameter);
    }
    done();
}

static void semihost_args_task(void *pvParameter)
{
    int ret;
    int core_id = xPortGetCoreID();

    /**** Init ****/
    ESP_LOGI(TAG, "CPU[%d]: Initialization", core_id);
    if (core_id == 0) {
        ret = esp_vfs_semihost_register("/host", NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to register semihost driver (%s)!", core_id, esp_err_to_name(ret));
            return;
        }
#if !CONFIG_FREERTOS_UNICORE
        xTaskCreatePinnedToCore(&semihost_args_task, "semihost_args_task1", 8000, xTaskGetCurrentTaskHandle(), 5, NULL, 1);
        vTaskDelay(1);
#endif
    }

    /**** Wrong args testing****/
    int wrong_args[] = {
        0xFFFFFFF0,
        0xFFFFFFFF
    };
    for (int i = 0; i < (sizeof(wrong_args) / sizeof(wrong_args[0])); i++) {
        semihosting_wrong_args(wrong_args[i]);
    }

    /***** De-init *****/
    if (core_id == 0) {
#if !CONFIG_FREERTOS_UNICORE
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
#endif
        ESP_LOGI(TAG, "CPU[%d]: Unregister host FS", core_id);
        ret = esp_vfs_semihost_unregister("/host");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to unregister semihost driver (%s)!", core_id, esp_err_to_name(ret));
            return;
        }
    } else {
        xTaskNotifyGive((TaskHandle_t)pvParameter);
    }
    done();
}
#endif /* #if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0) */

ut_result_t semihost_test_do(int test_num)
{
    switch (test_num) {
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0)
        case 700: {
        /*
        * *** About the test ***
        *
        * N - number of cores
        *
        * Its sequence:
        * - There are N (test_read.*) files containing random bytes
        * - Test creates N (test_write.*) files
        * - Test opens the read- and write-files
        * - Test writes (test_read.*) content to (test_write.*)
        * - Test closes the files
        */
            xTaskCreatePinnedToCore(&semihost_task, "semihost_task0", 4096, NULL, 5, NULL, 0);
            break;
        }
        case 701: {
        /*
        * *** About the test ***
        *
        * N - number of cores
        *
        * There are N (test_read.*) files containing random bytes
        * There are several wrongs arguments.
        *
        * For each of args for doing following:
        * - Send wrong syscall
        * - Send SYS_DRVINFO syscall with wrong args
        * - Send SYS_OPEN syscall with wrong args
        * - Open test_read.N file
        * - Send SYS_WRITE syscall with wrong args
        * - Send SYS_READ syscall with wrong args
        * - Send SYS_SEEK syscall with wrong args
        * - Send SYS_CLOSE syscall with wrong args
        * - Close the file
        */
        xTaskCreatePinnedToCore(&semihost_args_task, "semihost_args_task0", 8000, NULL, 5, NULL, 0);
        break;
    }
#endif /* #if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0) */
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}
