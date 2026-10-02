# OSS_AAP
OSS 과제 AAP 코드

## Build and Run (from `script.txt`)

The repository includes `script.txt` with the canonical build/run steps used by the project. Below is a consolidated, slightly annotated version suitable for this repository.

1. Configure and build (creates `build/`):

```sh
sudo -E cmake -B build .
sudo cmake --build build --target install -j $(nproc)
```

2. Run the runtime launcher installed by the build:

```sh
sudo -E build/install/para-exec.sh
```

3. (Optional) Entire run with full logging saved to `total_log.txt`:

```sh
{
  sudo -E cmake -B build .
  sudo cmake --build build --target install -j $(nproc)
  sudo -E build/install/para-exec.sh
} >&1 | tee total_log.txt

# or run with debug log level:
{
  export PARA_LOG_LEVEL=debug
  sudo -E build/install/para-exec.sh
} >&1 | tee total_log.txt
```

Notes and environment setup referenced in `script.txt`:
- The script mentions sourcing `/opt/para/para-env-setup.sh`. If your environment already includes Para SDK setup system-wide, this step may be unnecessary.
- Network route addition (`sudo ip route add 224.0.0.0/4 dev <if>`) and link state setup are environment-specific; the netplan in this system already configures the multicast route and interface state for the usual development machine.
- If needed, ensure dynamic library loading includes `/opt/para/lib`:

```sh
export LD_LIBRARY_PATH=/opt/para/lib:$LD_LIBRARY_PATH
echo "/opt/para/lib" | sudo tee /etc/ld.so.conf.d/para.conf
sudo ldconfig
```

Security note: the build and runtime steps in `script.txt` run parts as `sudo` and may install or run system-level services — review before executing on production machines.

If you want, I can also add a non-sudo developer-friendly flow (using a local prefix) or a CI job that performs these steps.
