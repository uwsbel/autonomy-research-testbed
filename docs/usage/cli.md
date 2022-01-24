# CLI Documentation

```{raw} html
---
---

<style>
	h4 {text-transform: lowercase;}
</style>
```

## `miniav`

```{autosimple} miniav._miniav_base._init
```

```{argparse}
---
module: miniav._miniav_base
func: _init
prog: miniav
nosubcommands:
nodescription:
---
```

## Sub-commands

Subcommands immediately succeed the `miniav` command. They implement additional logic. Having subcommands rather than arguments directly to `miniav` increases expandability as it will allow for additional features to be implemented without convoluting the help menu of the base `miniav` command.

### `dev`

```{autosimple} miniav.dev._init
```

```{argparse}
---
module: miniav._miniav_base
func: _init
prog: miniav
path: dev
nosubcommands:
nodescription:
---
```

#### `dev env`

```{autosimple} miniav.dev._run_env
```

```{argparse}
---
module: miniav._miniav_base
func: _init
prog: miniav
path: dev env 
nosubcommands:
nodescription:
---
```

### `db`

```{autosimple} miniav.db._init
```

```{argparse}
---
module: miniav._miniav_base
func: _init
prog: miniav
path: db
nosubcommands:
nodescription:
---
```

#### `db combine`

```{autosimple} miniav.db._run_combine
```

```{argparse}
---
module: miniav._miniav_base
func: _init
prog: miniav
path: db combine 
nosubcommands:
nodescription:
---
```

#### `db read`

```{autosimple} miniav.db._run_read
```

```{argparse}
---
module: miniav._miniav_base
func: _init
prog: miniav
path: db read 
nosubcommands:
nodescription:
---
```
