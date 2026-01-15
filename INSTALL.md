Acq4 : Installation with UV
===========================


1. Install `uv` on your system. See https://docs.astral.sh/uv/getting-started/installation/, or

    windows: 
    ```
    powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
    ```

    Mac OSX:
    ```
    curl -LsSf https://astral.sh/uv/install.sh | sh
    ```

2. Clone acq4 from the git repo from the Manis Lab. Create a directory where
you want acq4 to reside. I usually make a directory on the desktop as that makes it most obvious, but you may want a different structure. Go to that directory in the terminal. Then clone the git repo:
   
   ```
   git clone https://github.com/pbmanis/acq4.git
   
   ```

3. Jump into the acq4 directory (in the terminal), and checkout the current "working" branch that has the lab's latest changes. Eventually, I will make a branch called "ManisLab", which has my latest changes. Note that my version has diverged from the main branch, although it has most of the more recent changes. This version is probably not optimal for acquisition as it has not been extensively tested (for that there are other branches available).
   
4. At this point, I am usually in VSCode, and using the terminal there. Make sure that VSCode is in the acq4 directory - this makes it easy to see what is going on, to invoke scripts, and to adjust the configuration files. 

    ```
    cd acq4
    git checkout use-UV-catchup
    ```

5. Now build/create the working environment. This will exists in a directory called ".venv" in the main acq4 directory. 
   
   ```
   uv venv
   uv venv --python 3.13.7   # the version of python to use
   uv sync   # get all the external libraries/modules acq4 needs to run

6. Start acq4:

   ```
   source .venv/bin/activate.bat  # or what is needed to activate the environment
   python -m acq4

   ```
    The first time acq4 is run, it may take some time to build stuff, but after that it will be ready to use.

7. If you want to have a way to autostart acq4, you will have to figure it out. On windows, a shortcut might work (or run a .bat file).

8. When you are done with acq4, it is potentially a good idea to deactivate the environment. Note that if you are running in a vscode terminal, closing the terminal will do this for you.

    ```
    deactivate
    ```






