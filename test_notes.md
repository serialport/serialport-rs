I'm seeing line processing behavior on Linux with my current implementation. 3.3, 4.0, and better_modes all have the same behavior.
I have to send \n characters to get data to be read immediately. If I eventually fill up the buffer (haven't figured out the exact size yet)
then data will also be output.
`moserial` doesn't have the same issues with reading singular bytes. I can't imagine it's in my libray, but I might need to dissect mine versus
continuing to investigate POSIX termios settings that aren't right. I copied the configuration settings done by moserial, but I'm still getting
the same issues.
