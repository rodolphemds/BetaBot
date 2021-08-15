#!/bin/bash

begin()
{
echo "Updating OS..."
}

apt_update()
{
apt-get update;
apt-get upgrade -y;
apt-get dist-upgrade -y;
}

cleaning()
{
apt-get autoremove -y;
apt-get autoclean -y;
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