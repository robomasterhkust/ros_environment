/**
 * \file COMPort_Windows.c
 *
 * \brief This file implements the API to access a serial communication port
 *        for Windows.
 *
 * \see COMPort.h for details
 */

/* ===========================================================================
** Copyright (C) 2016-2017 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorisation.
** ===========================================================================
*/

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/
#include "COMPort.h"

#if defined _WIN32

#include "Windows.h"
#include <stdio.h>

/*
==============================================================================
   4. DATA
==============================================================================
*/
static HANDLE* handles = NULL;            /**< An array of handles to open COM
                                               ports. */
static size_t num_allocated_handles = 0;  /**< The current size of the handle
                                               array. */
static size_t num_open_handles = 0;       /**< The current number of open
                                               handles. */

/*
==============================================================================
   7. EXPORTED FUNCTIONS
==============================================================================
*/

uint32_t com_get_port_list(char* port_list, size_t buffer_size)
{
	uint32_t num_available_ports = 0;
	uint32_t i;

	size_t available_size = buffer_size;

	/* init port list to empty string */
	if (buffer_size > 0)
		port_list[0] = 0;

    /*
     * Up to 255 COM ports are supported, but Windows does not allow to check
     * which ones are available. So try to open all ports get the default
     * configuration. If this succeeds, the port is available.
     */
    for (i = 1; i < 256; ++i)
    {
		char port_name[7] = "";
		char portDescr[200] = "";

		COMMCONFIG port_configuration;
		DWORD size;
		
        /* build name of current COM port */
        sprintf(port_name, "COM%i", i);

        /* try to access COM port */
        size = sizeof(COMMCONFIG);

		if (QueryDosDevice(port_name, portDescr, 200))
		{
			size_t name_length;

            /* add the port name to the list, if buffer capacity is
             * sufficient
             */
            name_length = strlen(port_name);

            if (num_available_ports > 0)
            {
                /* add one for the separator */
                ++name_length;
            }

			if (name_length < available_size)
            {
                if (num_available_ports > 0)
                    strcat_s(port_list, buffer_size, ";");

                strcat_s(port_list, buffer_size, port_name);

                /* update ramaining capacity of string buffer */
				available_size -= name_length;
            }

            /* port is availabe, so increase counter */
            ++num_available_ports;
        }
    }

    return num_available_ports;
}

int32_t com_open(const char* port_name)
{
    HANDLE com_port_handle;
    COMMTIMEOUTS timeouts;
    DCB com_configuration = {0};
    int32_t new_handle;

    /*
     * use full path because Windows won't find "COM10" and higher
     * (see knowledgebase http://support.microsoft.com/kb/115831/en-us)
     */
    char full_port_name[20] = "\\\\.\\";
    if (strlen(full_port_name) + strlen(port_name) < sizeof(full_port_name))
        strcat(full_port_name, port_name);

    /* open the COM port */
    /* ----------------- */
    com_port_handle = CreateFile(full_port_name,  /* name of the COM port */
                                 GENERIC_READ |   /* access type */
                                 GENERIC_WRITE,
                                 0,               /* shared mode */
                                 NULL,            /* security attributes */
                                 OPEN_EXISTING,   /* creation disposition */
                                 0,               /* flags and attributes */
                                 0);              /* template file */

    /* if COM port could not be openend, return negative Windows error code */
    if (com_port_handle == INVALID_HANDLE_VALUE)
        return -(int)GetLastError();

    /* set timeouts */
    timeouts.ReadIntervalTimeout         = 0;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 1000;
    timeouts.WriteTotalTimeoutConstant   = 100;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    SetCommTimeouts(com_port_handle, &timeouts);

    /* configure COM Port (even though it's virtual) */
    com_configuration.DCBlength = sizeof(DCB);
    GetCommState (com_port_handle, &com_configuration);
    com_configuration.BaudRate = 115200;
    com_configuration.ByteSize = 8;
    com_configuration.StopBits = ONESTOPBIT;
    SetCommState (com_port_handle, &com_configuration);

    /* add the handle to the array of open COM ports */
    /* --------------------------------------------- */

    /* increase capacity of handle array if needed */
    if (num_open_handles == num_allocated_handles)
    {
        HANDLE* new_handles = (HANDLE*)malloc((num_open_handles + 1) *
                                              sizeof(HANDLE));

        if (num_allocated_handles > 0)
        {
            memcpy(new_handles, handles, sizeof(HANDLE) *
                                         num_allocated_handles);
        }
        new_handles[num_allocated_handles++] = INVALID_HANDLE_VALUE;

        free(handles);
        handles = new_handles;
    }

    /* add new handle to table */
    new_handle = 0;
    while (handles[new_handle] != INVALID_HANDLE_VALUE)
        ++new_handle;

    handles[new_handle] = com_port_handle;
    ++num_open_handles;

    return new_handle;
}

void com_close(int32_t port_handle)
{
    /* check if handle is valid */
    if ((port_handle >= 0) &&
        (port_handle < (int32_t)num_allocated_handles) &&
        (handles[port_handle] != INVALID_HANDLE_VALUE))
    {
        /* stop all transfers are in progress */
        CancelIo(handles[port_handle]);

        /* close COM port */
        CloseHandle(handles[port_handle]);

        /* remove handle from the table */
        handles[port_handle] = INVALID_HANDLE_VALUE;
        --num_open_handles;

        /* if all handles have been closed, free handle table to prevent
         * memory leaks
         */
        if (num_open_handles == 0)
        {
            num_allocated_handles = 0;
            free(handles);
            handles = NULL;
        }
    }
}

void com_send_data(int32_t port_handle, const void* data, size_t num_bytes)
{
    /* check if handle is valid */
    if ((port_handle >= 0) &&
        (port_handle < (int32_t)num_allocated_handles) &&
        (handles[port_handle] != INVALID_HANDLE_VALUE))
    {
        /* send data */
        DWORD num_bytes_written;
        WriteFile(handles[port_handle], data, num_bytes,
                  &num_bytes_written, NULL);
    }
}

size_t com_get_data(int32_t port_handle,
                    void* data, size_t num_requested_bytes)
{
    /* check if handle is valid */
    if ((port_handle >= 0) &&
        (port_handle < (int32_t)num_allocated_handles) &&
        (handles[port_handle] != INVALID_HANDLE_VALUE))
    {
        /* read data */
        DWORD num_bytes_read = 0;

        ReadFile(handles[port_handle], data, num_requested_bytes,
                 &num_bytes_read, NULL);

        return num_bytes_read;
    }

    /* if handle was not valid, return 0 to indicate that no data was
     * received
     */
    return 0;
}

void com_set_timeout(int32_t port_handle, uint32_t timeout_period_ms)
{
    /* check if handle is valid */
    if ((port_handle >= 0) &&
        (port_handle < (int32_t)num_allocated_handles) &&
        (handles[port_handle] != INVALID_HANDLE_VALUE))
    {
        /* set timeouts */
        COMMTIMEOUTS timeouts;
        timeouts.ReadIntervalTimeout         = 0;
        timeouts.ReadTotalTimeoutMultiplier  = 0;
        timeouts.ReadTotalTimeoutConstant    = timeout_period_ms;
        timeouts.WriteTotalTimeoutConstant   = 100;
        timeouts.WriteTotalTimeoutMultiplier = 1;
        SetCommTimeouts(handles[port_handle], &timeouts);
    }
}

/* --- Close open blocks -------------------------------------------------- */

/* End of Windows only code */
#endif /* _WIN32 */

/* --- End of File -------------------------------------------------------- */
