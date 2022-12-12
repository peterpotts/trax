name := "trax"

organization := "com.peterpotts"

version := "1.0.0-SNAPSHOT"

homepage := Some(url("https://github.com/peterpotts/trax"))

startYear := Some(2015)

scmInfo := Some(
  ScmInfo(
    url("https://github.com/peterpotts/trax"),
    "scm:git:https://github.com/peterpotts/trax.git",
    Some("scm:git:git@github.com:peterpotts/trax.git")))

scalaVersion := "2.13.1"

scalacOptions ++= Seq(
  "-unchecked",
  "-deprecation",
  "-encoding",
  "UTF-8",
  "-feature",
  "-language:postfixOps",
  "-Xlint:_",
  "-Xverify")

javacOptions ++= Seq(
  "-Xlint:unchecked",
  "-Xlint:deprecation"
)

fork := true

javaOptions in run += "-Djava.library.path=src/main/resources"

libraryDependencies ++= {
  object Versions {
    val scalafx = "12.0.2-R18"
    val scalaz = "7.2.27"
    val scalaTest = "3.2.0"
    val mockito = "1.10.19"
    val akka = "2.6.5"
    val jinput = "2.0.9"
    val jodaConvert = "1.8"
    val jodaTime = "2.9.3"
    val typesafeConfig = "1.2.1"
    val slf4j = "1.7.21"
    val logback = "1.1.3"
    val mavlink = "1.1.8"
  }

  object Dependencies {
    val scalaCompiler = "org.scala-lang" % "scala-compiler" % scalaVersion.value
    val scalafx = "org.scalafx" %% "scalafx" % Versions.scalafx
    val scalaz = "org.scalaz" %% "scalaz-core" % Versions.scalaz
    val scalaTest = "org.scalatest" %% "scalatest" % Versions.scalaTest

    val mockitoCore = "org.mockito" % "mockito-core" % Versions.mockito

    val akkaActor = "com.typesafe.akka" %% "akka-actor-typed" % Versions.akka

    val jinput = "net.java.jinput" % "jinput" % Versions.jinput

    val jodaConvert = "org.joda" % "joda-convert" % Versions.jodaConvert
    val jodaTime = "joda-time" % "joda-time" % Versions.jodaTime

    val typesafeConfig = "com.typesafe" % "config" % Versions.typesafeConfig

    val mavlink = "io.dronefleet.mavlink" % "mavlink" % Versions.mavlink

    val slf4jApi = "org.slf4j" % "slf4j-api" % Versions.slf4j
    // log4jOverSlf4j: Associated with exclude("log4j", "log4j")
    val log4jOverSlf4j = "org.slf4j" % "log4j-over-slf4j" % Versions.slf4j
    // jclOverSlf4j: Associated with exclude("commons-logging", "commons-logging")
    val jclOverSlf4j = "org.slf4j" % "jcl-over-slf4j" % Versions.slf4j
    val logbackClassic = "ch.qos.logback" % "logback-classic" % Versions.logback
  }

  import Dependencies._

  Seq(
    scalaCompiler,
    scalafx,
    scalaz,
    scalaTest % "test",
    mockitoCore % "test",
    akkaActor,
    jinput,
    jodaConvert,
    jodaTime,
    typesafeConfig,
    mavlink,
    "javax.vecmath" % "vecmath" % "1.5.2",
    slf4jApi,
    log4jOverSlf4j,
    jclOverSlf4j,
    logbackClassic)
}

/*-----------------*/
/* Assembly plugin */
/*-----------------*/

mainClass in assembly := Some("com.peterpotts.trax.Application")

assemblyMergeStrategy in assembly := {
  case "reference.conf" => MergeStrategy.first
  case pathList => (assemblyMergeStrategy in assembly).value(pathList)
}
