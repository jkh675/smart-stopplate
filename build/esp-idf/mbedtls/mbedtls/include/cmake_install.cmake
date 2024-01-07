# Install script for directory: G:/DevEnv/espidf/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/stopplate_refact")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/Users/rluo2/.rustup/toolchains/esp/xtensa-esp32-elf/esp-12.2.0_20230208/xtensa-esp32-elf/bin/xtensa-esp32-elf-objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/build_info.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/compat-2.x.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/config_psa.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/constant_time.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/legacy_or_psa.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/lms.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/mbedtls_config.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/pkcs7.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/private_access.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/psa_util.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/psa" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_builtin_composites.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_builtin_primitives.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_compat.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_config.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_driver_common.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_composites.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_primitives.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_extra.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_platform.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_se_driver.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_sizes.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_struct.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_types.h"
    "G:/DevEnv/espidf/components/mbedtls/mbedtls/include/psa/crypto_values.h"
    )
endif()

