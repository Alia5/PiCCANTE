SECTIONS
{
    /* Place the filesystem at the end of the flash */
    .littlefs_fs :
    {
        . = ALIGN(4);
        __filesystem_start = .;
        KEEP(*(.littlefs_fs))
        __filesystem_end = .;
    } > LITTLEFS
}

/* Export the filesystem section symbols */
PROVIDE(__filesystem_start = ADDR(.littlefs_fs));
PROVIDE(__filesystem_end = ADDR(.littlefs_fs) + SIZEOF(.littlefs_fs));