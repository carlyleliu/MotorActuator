aux_source_directory(factory/Src FACTORY_SRCS)
aux_source_directory(sample/Src SAMPLE_SRCS)

zephyr_include_directories(factory/Inc)

zephyr_library_sources(
    ${FACTORY_SRCS}
)
