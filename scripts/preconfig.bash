# ====================================================================================================================================
# @file       preconfig.bash
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 12th May 2022 8:07:39 pm
# @modified   Thursday, 19th May 2022 6:23:58 pm
# @project    engineering-thesis
# @brief
#    
#    Preconfiguration of the project's workspace
#    
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# Add '~/.local/bin' to PATH
if ! echo "$PATH" | grep "$HOME/.local/bin" > /dev/null; then
    export PATH="$PATH:$HOME/.local/bin"
fi
