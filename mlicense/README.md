# MlLicense

### Generate keys and license.cpp for a new version
Goto `src` folder and change the `mlicense_config.json` file with new version information.
Then use the command:
```shell script
./mlicense_gen.py mlicense_config.json
```
This will generate a new key pair and send it to the server. 
Then it will replace the public key, version, and url in the `license.cpp` file.

### Future development
The `license.cpp` file can be edited directly and the `mlicense_gen.py` script will still work
as long as the `public_key`, `url`, and `version` is kept between the `/* begin <type> */` and `/* end <type> */`
(type refers to `public_key`, `url`, and `version`)