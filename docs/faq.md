# Frequently Asked Questions

## File Permissions

For Linux users, you may run into issues regarding file permissions when using docker.
By properly setting the user id (UID) and group id (GID) of the created user in the
image, these issues can usually be avoided. By default, in the `base.dockerfile` file,
the UID and GID are both set to 1000 (the default UID/GID for new users in most linux
distributions).

If you are running into file permission issues, you may want to try the following.

First, check the user id and group id with the following commands:

```bash
$ id -u
1001

$ id -g
1001
```

If you see something similar to the above, where the output id's are _not_ 1000, you
will need to update the `USER_UID` and `USER_GID` environment variables. It is
recommended this is done either through your host's profile file (e.g. `~/.bashrc` or
`~/.zshrc`) or assigning the variables in the `.env` file in the root of the
`autonomy-research-testbed` repo.

```bash
# In your ~/.bashrc or ~/.zshrc file
export USER_UID=1001
export USER_GID=1001
```

```bash
# In <path-to-art>/autonomy-research-testbed/.env
USER_UID=1001
USER_GID=1001
```

## OptiX Install

TODO
