@echo off
echo Opening door...
curl http://smart-door.local/button/open/press
echo Request complete.
timeout /t 3 /nobreak
exit