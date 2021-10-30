miniav -vv db combine -c data/cli_demo.yaml || exit 1
miniav -vv db read data/cli_demo.yaml || exit 1
