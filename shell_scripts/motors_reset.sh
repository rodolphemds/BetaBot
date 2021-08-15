#!/bin/bash

begin()
{
echo "Resetting connected motors..."
}

reset()
{
python2 /betabot/python_scripts/nxt1_reset.py;
python2 /betabot/python_scripts/nxt2_reset.py
}

end()
{
echo "Done."
exit
}

begin
reset
end