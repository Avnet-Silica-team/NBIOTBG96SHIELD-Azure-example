/**
 ******************************************************************************
 * @file    certificates.h
 * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
 * @brief   certificate used for mbedTLS
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
 
#ifndef __CERTIFICATES_H
#define __CERTIFICATES_H

const char certificates[] = 
/* Baltimore */
"-----BEGIN CERTIFICATE-----\r\n"
"MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\r\n"
"RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\r\n"
"VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\r\n"
"DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\r\n"
"ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\r\n"
"VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\r\n"
"mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\r\n"
"IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\r\n"
"mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\r\n"
"XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\r\n"
"dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\r\n"
"jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\r\n"
"BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\r\n"
"DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\r\n"
"9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\r\n"
"jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\r\n"
"Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\r\n"
"ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\r\n"
"R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\r\n"
"-----END CERTIFICATE-----\r\n"
/*WoSign*/
"-----BEGIN CERTIFICATE-----\r\n"
"MIIFdjCCA16gAwIBAgIQXmjWEXGUY1BWAGjzPsnFkTANBgkqhkiG9w0BAQUFADBV\r\n"
"MQswCQYDVQQGEwJDTjEaMBgGA1UEChMRV29TaWduIENBIExpbWl0ZWQxKjAoBgNV\r\n"
"BAMTIUNlcnRpZmljYXRpb24gQXV0aG9yaXR5IG9mIFdvU2lnbjAeFw0wOTA4MDgw\r\n"
"MTAwMDFaFw0zOTA4MDgwMTAwMDFaMFUxCzAJBgNVBAYTAkNOMRowGAYDVQQKExFX\r\n"
"b1NpZ24gQ0EgTGltaXRlZDEqMCgGA1UEAxMhQ2VydGlmaWNhdGlvbiBBdXRob3Jp\r\n"
"dHkgb2YgV29TaWduMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAvcqN\r\n"
"rLiRFVaXe2tcesLea9mhsMMQI/qnobLMMfo+2aYpbxY94Gv4uEBf2zmoAHqLoE1U\r\n"
"fcIiePyOCbiohdfMlZdLdNiefvAA5A6JrkkoRBoQmTIPJYhTpA2zDxIIFgsDcScc\r\n"
"f+Hb0v1naMQFXQoOXXDX2JegvFNBmpGN9J42Znp+VsGQX+axaCA2pIwkLCxHC1l2\r\n"
"ZjC1vt7tj/id07sBMOby8w7gLJKA84X5KIq0VC6a7fd2/BVoFutKbOsuEo/Uz/4M\r\n"
"x1wdC34FMr5esAkqQtXJTpCzWQ27en7N1QhatH/YHGkR+ScPewavVIMYe+HdVHpR\r\n"
"aG53/Ma/UkpmRqGyZxq7o093oL5d//xWC0Nyd5DKnvnyOfUNqfTq1+ezEC8wQjch\r\n"
"zDBwyYaYD8xYTYO7feUapTeNtqwylwA6Y3EkHp43xP901DfA4v6IRmAR3Qg/UDar\r\n"
"uHqklWJqbrDKaiFaafPz+x1wOZXzp26mgYmhiMU7ccqjUu6Du/2gd/Tkb+dC221K\r\n"
"mYo0SLwX3OSACCK28jHAPwQ+658geda4BmRkAjHXqc1S+4RFaQkAKtxVi8QGRkvA\r\n"
"Sh0JWzko/amrzgD5LkhLJuYwTKVYyrREgk/nkR4zw7CT/xH8gdLKH3Ep3XZPkiWv\r\n"
"HYG3Dy+MwwbMLyejSuQOmbp8HkUff6oZRZb9/D0CAwEAAaNCMEAwDgYDVR0PAQH/\r\n"
"BAQDAgEGMA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFOFmzw7R8bNLtwYgFP6H\r\n"
"EtX2/vs+MA0GCSqGSIb3DQEBBQUAA4ICAQCoy3JAsnbBfnv8rWTjMnvMPLZdRtP1\r\n"
"LOJwXcgu2AZ9mNELIaCJWSQBnfmvCX0KI4I01fx8cpm5o9dU9OpScA7F9dY74ToJ\r\n"
"MuYhOZO9sxXqT2r09Ys/L3yNWC7F4TmgPsc9SnOeQHrAK2GpZ8nzJLmzbVUsWh2e\r\n"
"JXLOC62qx1ViC777Y7NhRCOjy+EaDveaBk3e1CNOIZZbOVtXHS9dCF4Jef98l7VN\r\n"
"g64N1uajeeAz0JmWAjCnPv/So0M/BVoG6kQC2nz4SNAzqfkHx5Xh9T71XXG68pWp\r\n"
"dIhhWeO/yloTunK0jF02h+mmxTwTv97QRCbut+wucPrXnbes5cVAWubXbHssw1ab\r\n"
"R80LzvobtCHXt2a49CUwi1wNuepnsvRtrtWhnk/Yn+knArAdBtaP4/tIEp9/EaEQ\r\n"
"PkxROpaw0RPxx9gmrjrKkcRpnd8BKWRRb2jaFOwIQZeQjdCygPLPwj2/kWjFgGce\r\n"
"xGATVdVhmVd8upUPYUk6ynW8yQqTP2cOEvIo4jEbwFcW3wh8GcF+Dx+FHgo2fFt+\r\n"
"J7x6v+Db9NpSvd4MVHAxkUOVyLzwPt0JfjBkUO1/AaQzZ01oT74V77D2AhGiGxMl\r\n"
"OtzCWfHjXEa7ZywCRuoeSKbmW9m1vFGikpbbqsY3Iqb+zCB0oy2pLmvLwIIRIbWT\r\n"
"ee5Ehr7XHuQe+w==\r\n"
;

#endif /* __CERTIFICATES_H */
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

