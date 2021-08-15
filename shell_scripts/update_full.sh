#!/bin/bash

begin()
{
echo "Updating BetaBot system..."
python2 /betabot/python_scripts/nxt2_light_anim.py;
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

update_filesystem()
{
cd /betabot/shell_scripts/
sudo ./update_filesystem.sh
}

end()
{
python2 /betabot/python_scripts/nxt2_stop_program.py;
echo "BetaBot is now up-to-date."
exit
}

begin
apt_update
cleaning
update_filesystem
end