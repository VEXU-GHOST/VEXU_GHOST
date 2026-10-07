# Load the machine's robot name from one obvious, user-owned file.
ROBOT_NAME_FILE="$HOME/robot_name"

if [ -s "$ROBOT_NAME_FILE" ]; then
    IFS= read -r ROBOT_NAME < "$ROBOT_NAME_FILE"
    ROBOT_NAME="$(printf '%s' "$ROBOT_NAME" | tr '[:lower:]' '[:upper:]')"
    export ROBOT_NAME
else
    unset ROBOT_NAME
fi

export ROBOT_NAME_FILE
