#!/bin/bash

begin()
{
echo "Updating OS..."
}

apt_update()
{
sudo apt-get update;
sudo apt-get upgrade -y;
sudo apt-get dist-upgrade -y;
}

cleaning()
{
sudo apt-get autoremove -y;
sudo apt-get autoclean -y;
}

end()
{
echo "OS up-to-date."
exit
}

begin
apt_update
cleaning
end