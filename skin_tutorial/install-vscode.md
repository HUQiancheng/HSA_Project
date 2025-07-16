# Install Docker in Ubuntu

https://code.visualstudio.com/docs/setup/linux


Last updated on 04.07.2024. Please always check the instructions of the link 
above before pasting the commands below.


## Add the APT repository for vscode

```bash
sudo apt-get install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
rm -f packages.microsoft.gpg
```

## Install vscode


```bash
sudo apt install apt-transport-https
sudo apt update
sudo apt install code
```


## Install the dev container extension

https://code.visualstudio.com/docs/devcontainers/containers
https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers

```
# Press: CTRL+P and paste
ext install ms-vscode-remote.remote-containers
```