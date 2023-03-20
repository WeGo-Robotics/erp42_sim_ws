#!/bin/bash

# Get the current directory path
current_path=$(pwd)

# Change directory to the parent directory of the folders containing the Python files
cd "$current_path"

# Build work space
catkin_make
# Change permissions of all Python files in subdirectories
SHELL_TYPE="$(basename "$SHELL")"

if [ "$SHELL_TYPE" == "bash" ]; then
  # Bash shell
  if ! grep -q "source $current_path/devel/setup.bash" ~/.zshrc; then
    echo "source $current_path/devel/setup.bash" >> ~/.bashrc
    echo "Added 'source $current_path/devel/setup.bash' to ~/.bashrc"
  fi
elif [ "$SHELL_TYPE" == "zsh" ]; then
  # Zsh shell
	if ! grep -q "source $current_path/devel/setup.zsh" ~/.zshrc; then
    echo "source $current_path/devel/setup.zsh" >> ~/.zshrc
    echo "Added 'source $current_path/devel/setup.zsh' to ~/.zshrc"
  fi
else
  echo "Unsupported shell type: $SHELL_TYPE"
  exit 1
fi

sudo find . -name "*.py" -type f -exec chmod 777 {} \;

# Setup Completed

echo ""
echo "--------------------"
echo "Setup completed."
echo "--------------------"
echo ""