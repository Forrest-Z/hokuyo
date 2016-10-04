#ifndef _BOB_VISUALIZATION_COLOR_H_
#define _BOB_VISUALIZATION_COLOR_H_

namespace bob
{

class Color
{

public:

	virtual float r() const =0;
	virtual float g() const =0;
	virtual float b() const =0;

};

class Red : public Color
{

public:

	virtual float r() const { return 1.0f; };
	virtual float g() const { return 0.0f; };
	virtual float b() const { return 0.0f; };

};

class Green : public Color
{

public:

	virtual float r() const { return 0.0f; };
	virtual float g() const { return 1.0f; };
	virtual float b() const { return 0.0f; };

};

class Blue : public Color
{

public:

	virtual float r() const { return 0.0f; };
	virtual float g() const { return 0.0f; };
	virtual float b() const { return 1.0f; };

};

class Purple : public Color
{

public:

	virtual float r() const { return 1.0f; };
	virtual float g() const { return 0.0f; };
	virtual float b() const { return 1.0f; };

};

}

#endif
