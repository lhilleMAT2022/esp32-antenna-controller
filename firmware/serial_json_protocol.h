#pragma once

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "espnow_smoke_config.h"

namespace antenna_controller {

struct SerialRotatorCommand {
    uint8_t nodeId = 0;
    uint32_t sequence = 0;
    CommandType command = CommandType::None;
    float valueDegrees = 0.0F;
    uint32_t executeAtEpochSeconds = 0;
};

inline const char* jsonValue(const char* line, const char* key) {
    char token[24]{};
    snprintf(token, sizeof(token), "\"%s\"", key);
    const char* value = strstr(line, token);
    if (value == nullptr) {
        return nullptr;
    }
    value += strlen(token);
    while (*value == ' ' || *value == '\t') {
        ++value;
    }
    if (*value++ != ':') {
        return nullptr;
    }
    while (*value == ' ' || *value == '\t') {
        ++value;
    }
    return value;
}

inline bool jsonUnsigned(const char* line,
                         const char* key,
                         uint32_t* result) {
    const char* value = jsonValue(line, key);
    if (value == nullptr || *value < '0' || *value > '9') {
        return false;
    }
    char* end = nullptr;
    const unsigned long parsed = strtoul(value, &end, 10);
    if (end == value) {
        return false;
    }
    *result = static_cast<uint32_t>(parsed);
    return true;
}

inline bool jsonFloat(const char* line, const char* key, float* result) {
    const char* value = jsonValue(line, key);
    if (value == nullptr) {
        return false;
    }
    char* end = nullptr;
    const float parsed = strtof(value, &end);
    if (end == value) {
        return false;
    }
    *result = parsed;
    return true;
}

inline bool jsonString(const char* line,
                       const char* key,
                       char* result,
                       size_t resultLength) {
    const char* value = jsonValue(line, key);
    if (value == nullptr || *value++ != '"' || resultLength == 0) {
        return false;
    }
    size_t length = 0;
    while (*value != '\0' && *value != '"' && length + 1 < resultLength) {
        result[length++] = *value++;
    }
    if (*value != '"') {
        return false;
    }
    result[length] = '\0';
    return true;
}

inline bool parseSerialRotatorCommand(const char* line,
                                      SerialRotatorCommand* result) {
    char type[8]{};
    char command[12]{};
    uint32_t nodeId = 0;
    if (!jsonString(line, "t", type, sizeof(type)) ||
        strcmp(type, "rc") != 0 || !jsonUnsigned(line, "n", &nodeId) ||
        nodeId < 1 || nodeId > AntennaNodeCount ||
        !jsonUnsigned(line, "q", &result->sequence) ||
        !jsonString(line, "c", command, sizeof(command))) {
        return false;
    }

    result->nodeId = static_cast<uint8_t>(nodeId);
    if (strcmp(command, "goto") == 0) {
        result->command = CommandType::SetAzimuth;
        return jsonFloat(line, "az", &result->valueDegrees) &&
               result->valueDegrees >= 0.0F &&
               result->valueDegrees < 360.0F;
    }
    if (strcmp(command, "step") == 0) {
        result->command = CommandType::StepAzimuth;
        return jsonFloat(line, "d", &result->valueDegrees);
    }
    if (strcmp(command, "queue") == 0) {
        result->command = CommandType::QueueAzimuth;
        return jsonFloat(line, "az", &result->valueDegrees) &&
               result->valueDegrees >= 0.0F &&
               result->valueDegrees < 360.0F &&
               jsonUnsigned(line, "at", &result->executeAtEpochSeconds);
    }
    if (strcmp(command, "overlap") == 0) {
        result->command = CommandType::SetOverlapSouth;
        return jsonFloat(line, "az", &result->valueDegrees);
    }
    if (strcmp(command, "stop") == 0) {
        result->command = CommandType::Stop;
        return true;
    }
    if (strcmp(command, "time") == 0) {
        result->command = CommandType::None;
        return jsonUnsigned(line, "at", &result->executeAtEpochSeconds);
    }
    return false;
}

inline const char* serialCommandName(CommandType command) {
    switch (command) {
        case CommandType::SetAzimuth:
            return "goto";
        case CommandType::StepAzimuth:
            return "step";
        case CommandType::QueueAzimuth:
            return "queue";
        case CommandType::SetOverlapSouth:
            return "overlap";
        case CommandType::Stop:
            return "stop";
        default:
            return "time";
    }
}

}  // namespace antenna_controller
