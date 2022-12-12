package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;

import java.io.IOException;

/**
 * User: ton Date: 02.12.13 Time: 20:56
 */
public abstract class MAVLinkPort {
    public MAVLinkSchema schema;

    protected MAVLinkPort(MAVLinkSchema schema) {
        this.schema = schema;
    }

    public abstract void open() throws IOException;

    public abstract void close() throws IOException;

    public abstract void write(MAVLinkMessage message) throws IOException;

    public abstract MAVLinkMessage read() throws IOException;
}
