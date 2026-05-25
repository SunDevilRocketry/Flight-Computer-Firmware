#
# Emulator Integration Test Runner
#

export BASE_DIR="../.."

# Checks for the required environment variables
function env_check() {
    REQUIRED_VARS=("SDEC_BASE" "SDEC_COMPORT" "EMULATOR_COMPORT")
    echo "Validating environment variables."
    for var in "${REQUIRED_VARS[@]}"; do
        if [[ -z "${!var}" ]]; then
            echo "Error: $var is not set."
            echo "Please set a base path for SDEC, a comport for SDEC, and the other end for the emulator"
            return 1
        fi
    done
    return 0
}

env_check