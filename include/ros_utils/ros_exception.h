/**
 * @file RosException.h
 * @brief Defines custom exception classes for ROS-related errors.
 *
 * This header file defines a set of custom exception classes tailored for handling ROS-related errors.
 * The exceptions are designed to be used in ROS (Robot Operating System) applications to provide
 * meaningful error information and enhance error handling.
 *
 * @note This code is released under the MIT License.
 *       Copyright (c) 2023 Julian Rendon (julianrendon514@gmail.com)
 *
 * @version 1.0.0
 * @date 2023-09-12
 * @author Julian Rendon
 */

#ifndef ROS_EXCEPTION_H
#define ROS_EXCEPTION_H

#include <stdexcept>
namespace RosException
{
/**
 * @brief Enumeration of error codes for ROS exceptions.
 */
enum class ErrorCode
{
    PARAM_NOT_FOUND, /**< Parameter not found error. */
    INVALID_PARAM,   /**< Invalid parameter error. */
    UNKOWN_ERROR     /**< Unknown error. */
};

/**
 * @brief Base class for custom ROS exceptions.
 */
class Exception : public std::exception
{
  protected:
    std::string errorMessage; /**< Error message associated with the exception. */
    ErrorCode errorCode;      /**< Error code associated with the exception. */

  public:
    /**
     * @brief Constructor for creating a custom exception.
     * @param message The error message.
     * @param code The error code.
     */
    Exception(const char *message, ErrorCode code) : errorMessage(message), errorCode(code) {}

    /**
     * @brief Get a C-style string describing the exception.
     * @return A C-style string describing the exception.
     */
    const char *what() const noexcept override { return errorMessage.c_str(); }

    /**
     * @brief Get the error code associated with the exception.
     * @return The error code.
     */
    ErrorCode code() const noexcept { return errorCode; }
};

/**
 * @brief Custom exception class for representing invalid parameter errors.
 */
class InvalidParameter : public Exception
{
  public:
    /**
     * @brief Constructor for creating an InvalidParameter exception.
     */
    InvalidParameter() : Exception("invalid_parameter", ErrorCode::INVALID_PARAM) {}
};

/**
 * @brief Custom exception class for representing parameter not found errors.
 */
class ParameterNotFound : public Exception
{
  public:
    /**
     * @brief Constructor for creating a ParameterNotFound exception.
     */
    ParameterNotFound() : Exception("parameter_not_found", ErrorCode::PARAM_NOT_FOUND) {}
};

/**
 * @brief Custom exception class for representing unknown errors.
 */
class UnkownError : public Exception
{
  public:
    /**
     * @brief Constructor for creating an UnkownError exception.
     */
    UnkownError() : Exception("unknown_error", ErrorCode::UNKOWN_ERROR) {}
};

}   // namespace RosException

#endif