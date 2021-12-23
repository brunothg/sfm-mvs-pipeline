#!/usr/bin/env groovy
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Paths
import java.util.concurrent.atomic.AtomicLong
import java.util.stream.LongStream

println()
println(this.class.getSimpleName() + ' ' + args.findAll({ !it.isBlank() }).join(' '))

def cli = new CliBuilder(
        usage: "${this.class.getSimpleName()} -i <input file> -pw <pixel width> [<options>]"
        , header: 'Options:'
        , width: 80
        , stopAtNonOption: false
)
cli.with {
    i(longOpt: 'input', args: 1, argName: 'file', 'Input file')
    o(longOpt: 'output', args: 1, argName: 'file', 'Output file')
    p(longOpt: 'pixel-width', args: 1, argName: 'number', 'Pixel Width')
    tx(longOpt: 'texture', args: 1, argName: 'texture file', 'Write texture coordinates for file')
    t(longOpt: 'triangles', 'Use triangles instead of quads')
    f(longOpt: 'invert-faces', 'Change direction of faces')
    mx(longOpt: 'mirror-x', 'Mirror x-axis')
    my(longOpt: 'mirror-y', 'Mirror y-axis')
    mz(longOpt: 'mirror-z', 'Mirror z-axis')
    c(longOpt: 'center', 'Set center to 0/0/0')
    b(longOpt: 'binary', 'Binary PLY Output')
    h(longOpt: 'help', 'Print this help text')
}

def options = cli.parse(args)
assert options

if (options.h) {
    cli.usage()
    System.exit(0)
}

if (!options.i) {
    println('No input file specified')
    System.exit(1)
}

if (!options.p) {
    println('No pixel width specified')
    System.exit(1)
}

def input = Paths.get((String) options.i).toAbsolutePath()
println('In: ' + input)

def output = (options.o) ? Paths.get((String) options.o).toAbsolutePath() : input.getParent().resolve("${input.getFileName()}.ply")
println('Out: ' + output)

def texture = (options.tx) ? Paths.get((String) options.tx).toAbsolutePath() : null

def tmp = output.getParent().resolve("${output.getFileName()}.tmp")
println()

println("Pixel-Width: ${options.p}")
println('Pixel-Height: auto')
println('Position: ' + ((options.c) ? 'Centered' : 'Unchanged'))
println('Triangualtion: ' + ((options.t) ? 'Triangles' : 'Quads'))
println('Faces: ' + ((options.f) ? 'Inverted' : 'Normal'))
println("Mirror[x,y,z]: [${options.mx}, ${options.my}, ${options.mz}]")
println("Texture: ${options.tx}")
println('Encoding: ' + ((options.b) ? 'Binary' : 'ASCII'))
println()

if (options.tx && !options.t) {
    println('Warning - texture with quads may not be supported (e.g. Meshlab)')
}

def pointCount = new AtomicLong(0)
def faceCount = new AtomicLong(0)
def width = Long.valueOf((String) options.p)
def height = 0L

new BufferedOutputStream(Files.newOutputStream(tmp)).withCloseable({ tmpOutStream ->
    def tmpWriter = new OutputStreamWriter(tmpOutStream, StandardCharsets.US_ASCII)

    println('Read XYZ-Points')
    Files.newBufferedReader(input, StandardCharsets.UTF_8).withCloseable({ inStream ->
        inStream.lines().with({ pointLines ->
            try {
                pointCount.set(pointLines.parallel().count())
            } finally {
                pointLines.close()
            }
        })
    })
    height = (long)(pointCount.get() / width)

    def translate = [0, 0, 0] as Double[]
    if (options.c) {
        Files.newBufferedReader(input, StandardCharsets.UTF_8).withCloseable({ inStream ->
            inStream.lines().with({ pointLines ->
                try {
                    def centerPointIdx = (((width / 2) + (width * (height / 2))) - 1) as long
                    def centerPoint = Arrays.stream(pointLines.sequential()
                            .skip(centerPointIdx).findFirst().orElse('0 0 0').split('\\s+'))
                            .map({ Double.valueOf(it) })
                            .toArray({ new Double[it] })
                    translate[0] = -centerPoint[0]
                    translate[1] = -centerPoint[1]
                    translate[2] = -centerPoint[2]
                } finally {
                    pointLines.close()
                }
            })
        })
    }

    Files.newBufferedReader(input, StandardCharsets.UTF_8).withCloseable({ inStream ->
        inStream.lines().with({ pointLines ->
            try {
                pointLines.sequential()
                        .map({ pointLine ->
                            def pointParts = pointLine.split('\\s+')
                            Arrays.stream(pointParts)
                                    .map({ Double.valueOf(it) })
                                    .toArray({ new Double[it] })
                        })
                        .forEachOrdered({ point ->
                            point[0] += translate[0]
                            point[1] += translate[1]
                            point[2] += translate[2]

                            point[0] *= (options.mx) ? -1 : 1
                            point[1] *= (options.my) ? -1 : 1
                            point[2] *= (options.mz) ? -1 : 1

                            if (options.b) {
                                def buf = ByteBuffer.allocate(3 * 4)
                                buf.order(ByteOrder.BIG_ENDIAN)

                                point.each({ buf.putFloat((float) it) })
                                buf.flip()
                                def bytes = new byte[buf.remaining()]
                                buf.get(bytes)
                                tmpOutStream.write(bytes)
                            } else {
                                tmpWriter.write(point.join(' ') + System.lineSeparator())
                            }

                        })
            } finally {
                pointLines.close()
            }
        })
    })

    System.out.println('Create faces')
    def progress = new AtomicLong(0)
    def faceLimit = pointCount.get() - width
    def progressThread = new Thread({
        while (!Thread.interrupted()) {
            def percent = (int) (((double) progress.get() / (double) faceLimit) * 100.0)
            synchronized (System.out) {
                print("\r${percent}%")
            }
            try {
                Thread.sleep(1000)
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt()
            }
        }
        println('\r100%')
    })
    progressThread.start()
    LongStream.range(0, faceLimit).parallel()
            .filter({ i ->
                long y = (long) ((long) i / (long) width)
                long x = i % width
                return !((y >= (height - 2)) || (x >= (width - 2)))
            })
            .forEach({ i ->
                def faces = new ArrayList<Long[]>(2)
                if (options.t) {
                    if (options.f) {
                        faces.add([i, i + 1, (i + width) + 1] as Long[])
                        faces.add([i, (i + width) + 1, (i + width)] as Long[])
                    } else {
                        faces.add([i, (i + width) + 1, i + 1] as Long[])
                        faces.add([i, (i + width), (i + width) + 1] as Long[])
                    }
                } else {
                    if (options.f) {
                        faces.add([i, i + 1, (i + width) + 1, (i + width)] as Long[])
                    } else {
                        faces.add([i, (i + width), (i + width) + 1, i + 1] as Long[])
                    }
                }

                for (def face in faces) {
                    def texCoords = new ArrayList<Double>(face.length * 2)
                    face.each({ idx ->
                        long y = (long) ((long) idx / (long) width)
                        long x = idx % width
                        texCoords.add((double) x / (double) width)
                        texCoords.add(1 - ((double) y / (double) height))
                    })

                    if (options.b) {
                        int faceBytes = 1 + (face.length * 4)
                        int texBytes = 1 + (texCoords.size() * 4)
                        def buf = ByteBuffer.allocate(faceBytes + texBytes)
                        buf.order(ByteOrder.BIG_ENDIAN)

                        buf.put((byte) face.length)
                        face.each({ buf.putInt((int) it) })

                        if (texture) {
                            buf.put((byte) texCoords.size())
                            texCoords.each({ buf.putFloat((float) it) })
                        }

                        buf.flip()
                        def bytes = new byte[buf.remaining()]
                        buf.get(bytes)
                        synchronized (tmpOutStream) {
                            tmpOutStream.write(bytes)
                        }
                    } else {
                        def faceString = face.length + ' ' + face.join(' ')
                        if (texture) {
                            faceString += ' ' + texCoords.size() + ' ' + texCoords.join(' ')
                        }
                        faceString += System.lineSeparator()
                        synchronized (tmpWriter) {
                            tmpWriter.write(faceString)
                        }
                    }

                    faceCount.getAndIncrement()
                }

                progress.getAndIncrement()
            })

    progressThread.interrupt()
    progressThread.join()
    tmpWriter.flush()

})

println('Write file')
Files.newOutputStream(output).withCloseable({ outStream ->
    def writer = new BufferedWriter(new OutputStreamWriter(outStream, StandardCharsets.US_ASCII))
    writer.writeLine('ply')
    writer.writeLine((options.b) ? 'format binary_big_endian 1.0' : 'format ascii 1.0')
    if (texture) {
        writer.writeLine("comment TextureFile ${texture.getFileName()}")
    }
    writer.writeLine("element vertex ${pointCount.get()}")
    ['x', 'y', 'z'].each({
        writer.writeLine("property float ${it}")
    })
    writer.writeLine("element face ${faceCount.get()}")
    writer.writeLine('property list uchar int vertex_indices')
    if (texture) {
        writer.writeLine('property list uchar float texcoord')
    }
    writer.writeLine('end_header')
    writer.flush()

    Files.copy(tmp, outStream)
    Files.deleteIfExists(tmp)

})
