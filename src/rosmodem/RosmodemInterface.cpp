class RosmodemInterface {
public:
  virtual char *read() = 0;
  virtual void write(char *chars) = 0;
}
