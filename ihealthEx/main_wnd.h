#pragma once
#include "UIlib.h"

#include "control_card.h"
#include "passive_mode.h"
#include "fatigue_test.h"
#include "Chart.h"

#ifdef _DEBUG
#	if defined(_UNICODE) || defined(UNICODE)
#       pragma comment(lib, "CChartu.lib")

#	else
#		pragma comment(lib, "CChart.lib")
#	endif
#else
#	if defined(_UNICODE) || defined(UNICODE)
#		pragma comment(lib, "CChart_u.lib")
#	else
#		pragma comment(lib, "CChart.lib")
#	endif
#endif

using namespace NsCChart;
using namespace DuiLib;

class CWndUI : public CControlUI {
public:
	CWndUI() : m_hWnd(NULL) {}

	virtual void SetInternVisible(bool bVisible = true) {
		__super::SetInternVisible(bVisible);
		::ShowWindow(m_hWnd, bVisible);
	}

	virtual void SetPos(RECT rc) {
		__super::SetPos(rc);
		::SetWindowPos(m_hWnd, NULL, rc.left, rc.top, rc.right - rc.left, rc.bottom - rc.top, SWP_NOZORDER | SWP_NOACTIVATE);
	}

	BOOL Attach(HWND hWndNew) {
		if (!::IsWindow(hWndNew)) {
			return FALSE;
		}

		m_hWnd = hWndNew;
		return TRUE;
	}

	HWND Detach() {
		HWND hWnd = m_hWnd;
		m_hWnd = NULL;
		return hWnd;
	}

protected:
	HWND m_hWnd;
};

class CMainWnd : public DuiLib::WindowImplBase {
//UI≥ı ºªØ
public:
	
	virtual LPCTSTR GetWindowClassName() const;
	virtual DuiLib::CDuiString GetSkinFile();
	void InitWindow();


public:
	void Notify(DuiLib::TNotifyUI &msg);
	virtual CControlUI *CreateControl(LPCTSTR pstrClassName);
	virtual void OnFinalMessage(HWND hWnd);
	virtual LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam);

private:
	CControlUI *InitializeChart();
	void OnDataUpdate(LPARAM lParam);
	virtual LRESULT OnKeyDown(UINT /*uMsg*/, WPARAM /*wParam*/, LPARAM /*lParam*/, BOOL& bHandled);
	
private:
	ControlCard *m_pControlCard = nullptr;
	FatigueTest *m_pFatigueTest = nullptr;

	DuiLib::CButtonUI *m_pInitialBtn;
	DuiLib::CButtonUI *m_pPositiveBtn;
	DuiLib::CButtonUI *m_pNegativeBtn;
	DuiLib::CButtonUI *m_pStopBtn;
	HWND m_UIWnd;
	CChartWnd m_ChartWnd;
};


