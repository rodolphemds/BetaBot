#!/bin/bash

begin()
{
echo "Updating BetaBot system..."
python2 /betabot/python_scripts/nxt2_light_anim.py;
}

update_os()
{
cd /betabot/shell_scripts/
sudo ./update_os.sh
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
update_os
update_filesystem
end