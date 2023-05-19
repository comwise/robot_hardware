## depend on these libs

- net-tools

when use system command `ifconfig`
       
- libcap2-bin
        
when use `getcap`, `setcap` in deployments
        
`sudo setcap cap_net_admin=+epi xxxxx` to give capability of net admin to program xxxxx

- libcap-dev

when use programmable `setcap`
